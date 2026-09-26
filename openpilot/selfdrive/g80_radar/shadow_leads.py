#!/usr/bin/env python3
"""
G80 v17 receive-only shadow leadOne/leadTwo verifier.

HARD SEPARATION:
- NEVER publishes radarState
- NEVER sends CAN
- NEVER modifies planner/MPC
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
import bisect
import math

OBJECT_MAX_AGE_NS = 350_000_000
PATH_MAX_AGE_NS = 350_000_000
VEGO_MAX_AGE_NS = 500_000_000
PRODUCTION_MAX_AGE_NS = 350_000_000

PATH_OVERLAP_HALF_M = 1.80
CUTIN_NEAR_HALF_M = 2.70
CUTIN_MAX_DREL_M = 80.0
CUTIN_MIN_DREL_M = 2.0
CUTIN_CONFIRM_S = 0.30
CUTIN_CLOSE_CONFIRM_S = 0.10
CUTIN_MIN_INWARD_RATE_MPS = 0.12
CUTIN_CLOSE_MIN_INWARD_RATE_MPS = 0.25
CUTIN_MIN_INWARD_DISP_M = 0.20
CUTIN_MIN_CONSISTENCY = 0.65
CUTIN_CLOSE_MIN_CONSISTENCY = 0.80

STATIONARY_MAX_VLEAD_MPS = 2.5
STATIONARY_CONFIRM_S = 0.25
MOVING_CONFIRM_S = 0.15
CAMERA_ONLY_MIN_PROB = 0.60
HISTORY_S = 1.60
HISTORY_STALE_S = 0.80
STATIONARY_SHADOW_MIN_GAP_M = 3.0

# Same-vehicle guard aligned with the upstream passenger-car footprint fusion.
DUP_DX_M = 4.8
DUP_DY_M = 2.1
DUP_DV_MPS = 3.0
LEAD_REID_MAX_AGE_S = 0.65
RADAR_ONLY_MAX_DREL_M = 100.0


def _finite(v, default=0.0):
  try:
    x = float(v)
    return x if math.isfinite(x) else default
  except Exception:
    return default


def _age_ok(recv_ns, now_ns, max_age_ns):
  try:
    age = int(now_ns) - int(recv_ns)
  except Exception:
    return False
  return -50_000_000 <= age <= max_age_ns


def extract_model_path(model, max_points=65):
  try:
    xs = list(model.position.x)
    ys = list(model.position.y)
  except Exception:
    return []
  n = min(len(xs), len(ys))
  if n <= 0:
    return []
  step = max(1, math.ceil(n / max_points))
  out = []
  for i in range(0, n, step):
    x = _finite(xs[i], math.nan)
    y = _finite(ys[i], math.nan)
    if math.isfinite(x) and math.isfinite(y):
      out.append((x, y))
  return out


def model_path_y_left(path, d_rel):
  if not path:
    return 0.0
  if len(path) == 1:
    return -_finite(path[0][1])
  xs = [_finite(p[0]) for p in path]
  i = bisect.bisect_left(xs, d_rel)
  if i <= 0:
    return -_finite(path[0][1])
  if i >= len(path):
    return -_finite(path[-1][1])
  x0, y0 = path[i - 1]
  x1, y1 = path[i]
  if abs(x1 - x0) < 1e-6:
    return -_finite(y0)
  t = (d_rel - x0) / (x1 - x0)
  return -(_finite(y0) + (_finite(y1) - _finite(y0)) * t)


def _lead_snapshot(lead):
  if lead is None:
    return {'status': False}
  try:
    if not bool(lead.status):
      return {'status': False}
  except Exception:
    return {'status': False}
  def g(name, default=0.0):
    try:
      return getattr(lead, name)
    except Exception:
      return default
  return {
    'status': True,
    'dRel': round(_finite(g('dRel')), 3),
    'yRel': round(_finite(g('yRel')), 3),
    'vRel': round(_finite(g('vRel')), 3),
    'vLead': round(_finite(g('vLead')), 3),
    'aLeadK': round(_finite(g('aLeadK')), 3),
    'modelProb': round(_finite(g('modelProb')), 4),
    'radar': bool(g('radar', False)),
    'radarTrackId': int(g('radarTrackId', -1)),
  }


def snapshot_production_radar_state(msg, recv_ns):
  """Copy Cap'n Proto radarState immediately into plain Python data."""
  if msg is None:
    return None
  try:
    log_ns = int(msg.logMonoTime)
    if not _age_ok(log_ns, recv_ns, PRODUCTION_MAX_AGE_NS):
      return None
    s = msg.radarState
    return {
      'recv_ns': int(recv_ns),
      'log_ns': log_ns,
      'leadOne': _lead_snapshot(s.leadOne),
      'leadTwo': _lead_snapshot(s.leadTwo),
    }
  except Exception:
    return None


@dataclass
class HistSample:
  t: float
  x: float
  y: float
  dpath: float
  vx: float | None


@dataclass
class Hist:
  samples: deque = field(default_factory=deque)
  last_seen: float = 0.0


class ShadowLeadVerifier:
  def __init__(self):
    self.hist: dict[str, Hist] = {}
    self.last_l1_key = None
    self.last_l2_key = None
    self.last_l1_state = None
    self.last_l2_state = None

  @staticmethod
  def _identity(o):
    return str(o.get('vehicle_key') or o.get('key') or o.get('front_key') or 'unknown')

  @staticmethod
  def _same_physical(a, b):
    if a is None or b is None:
      return False
    if abs(float(a['x']) - float(b['x'])) > DUP_DX_M or abs(float(a['y']) - float(b['y'])) > DUP_DY_M:
      return False
    av=a.get('vx'); bv=b.get('vx')
    if av is not None and bv is not None and abs(float(av)-float(bv)) > DUP_DV_MPS:
      return False
    return True

  @staticmethod
  def _reidentify(candidates, previous, now_s):
    """Keep lead identity through a radar/camera/corner key hand-over.

    The previous relative-x is propagated with vRel.  No stale lead is emitted;
    this is used only to select among *fresh* current candidates.
    """
    if not previous:
      return None
    dt=max(0.0,now_s-float(previous.get('t',now_s)))
    if dt > LEAD_REID_MAX_AGE_S:
      return None
    px=float(previous['x']) + float(previous.get('vx') or 0.0)*dt
    py=float(previous['y'])
    pv=previous.get('vx')
    best=None
    for c in candidates:
      dx=abs(float(c['x'])-px); dy=abs(float(c['y'])-py)
      if dx>DUP_DX_M or dy>DUP_DY_M:
        continue
      if pv is not None and c.get('vx') is not None and abs(float(c['vx'])-float(pv))>DUP_DV_MPS:
        continue
      cost=(dx/DUP_DX_M)**2+(dy/DUP_DY_M)**2
      if pv is not None and c.get('vx') is not None:
        cost += .35*(abs(float(c['vx'])-float(pv))/DUP_DV_MPS)**2
      if best is None or cost<best[0]:
        best=(cost,c)
    return None if best is None else best[1]

  @staticmethod
  def _fresh(o, now_ns):
    return _age_ok(o.get('recv_ns', 0), now_ns, OBJECT_MAX_AGE_NS)

  def _update_history(self, objs, path, path_valid, now_s):
    for o in objs:
      key = self._identity(o)
      x = _finite(o.get('x'), math.nan)
      y = _finite(o.get('y'), math.nan)
      if not key or not math.isfinite(x) or not math.isfinite(y):
        continue
      dpath = y - model_path_y_left(path, x) if path_valid else y
      vx = o.get('vx')
      vx = None if vx is None else _finite(vx)
      h = self.hist.setdefault(key, Hist())
      h.samples.append(HistSample(now_s, x, y, dpath, vx))
      h.last_seen = now_s
      while h.samples and now_s - h.samples[0].t > HISTORY_S:
        h.samples.popleft()
    for k in list(self.hist):
      if now_s - self.hist[k].last_seen > HISTORY_STALE_S:
        self.hist.pop(k, None)

  def _motion(self, key):
    h = self.hist.get(key)
    if h is None or len(h.samples) < 2:
      return {'span_s':0.0,'inward_rate':0.0,'inward_disp':0.0,'consistency':0.0,
              'outward_rate':0.0,'outward_disp':0.0}
    s = list(h.samples)
    first, last = s[0], s[-1]
    span = max(1e-3, last.t - first.t)
    inward_disp = abs(first.dpath) - abs(last.dpath)
    inward_rate = inward_disp / span
    outward_disp = -inward_disp
    outward_rate = outward_disp / span
    good = 0
    total = 0
    for a, b in zip(s, s[1:]):
      if b.t <= a.t:
        continue
      total += 1
      if abs(b.dpath) <= abs(a.dpath) + 0.03:
        good += 1
    return {
      'span_s':span,'inward_rate':inward_rate,'inward_disp':inward_disp,
      'consistency':good/max(1,total),'outward_rate':outward_rate,'outward_disp':outward_disp
    }

  @staticmethod
  def _evidence(o):
    cam = bool(o.get('camera_confirmed'))
    scc = bool(o.get('scc_teacher_confirmed'))
    front = bool(o.get('front_link')) or o.get('source') == 'front_track'
    return {
      'camera':cam,'scc':scc,'front':front,'rear_teacher':bool(o.get('teacher_match')),
      'cross_sensor':cam or scc or bool(o.get('front_link'))
    }

  @staticmethod
  def _lead_dict(c, role, reason, score):
    o = c['obj']
    return {
      'status':True,'validationState':'candidate','role':role,
      'key':c['key'],'sourceKey':o.get('key'),'vehicleKey':o.get('vehicle_key'),'source':o.get('source'),
      'dRel':round(c['x'],3),'yRel':round(c['y'],3),'dPath':round(c['dpath'],3),
      'vRel':None if c['vx'] is None else round(c['vx'],3),
      'vLead':None if c['vlead'] is None else round(c['vlead'],3),
      'modelProb':round(float(o.get('camera_prob',o.get('prob',0.0)) or 0.0),4),
      'radar':o.get('source')!='c4_camera',
      'cameraConfirmed':bool(o.get('camera_confirmed')),
      'sccConfirmed':bool(o.get('scc_teacher_confirmed')),
      'frontLinked':bool(o.get('front_link')),
      'teacherMatch':bool(o.get('teacher_match')),
      'stationary':None if c['stationary'] is None else bool(c['stationary']),
      'cutInConfirmed':bool(c.get('cutin_confirmed',False)),
      'cutInScore':round(float(c.get('cutin_score',0.0)),3),
      'cutOutScore':round(float(c.get('cutout_score',0.0)),3),
      'historyS':round(float(c['motion']['span_s']),3),
      'inwardRate':round(float(c['motion']['inward_rate']),3),
      'ageMs':round(float(c['age_ms']),1),
      'vehicleMemberCount':int(o.get('vehicle_member_count',1) or 1),
      'vehicleClusterKeys':list(o.get('vehicle_cluster_keys',[o.get('key')])) if isinstance(o.get('vehicle_cluster_keys',[o.get('key')]),list) else [o.get('key')],
      'reidentified':bool(c.get('reidentified',False)),
      'reason':reason,'score':round(float(score),3),
      'controlConnected':False,
    }

  @staticmethod
  def _agree(shadow, stock):
    if not shadow or not shadow.get('status') or not stock or not stock.get('status'):
      return False
    return abs(float(shadow['dRel'])-float(stock['dRel'])) <= 5.0 and abs(float(shadow['yRel'])-float(stock['yRel'])) <= 2.0

  def update(self, objects, model_path, v_ego, now_ns,
             production=None, model_path_recv_ns=0, v_ego_recv_ns=0):
    now_s = float(now_ns) * 1e-9
    path_valid = bool(model_path) and _age_ok(model_path_recv_ns, now_ns, PATH_MAX_AGE_NS)
    v_ego_valid = _age_ok(v_ego_recv_ns, now_ns, VEGO_MAX_AGE_NS)
    production_valid = production is not None and _age_ok(production.get('recv_ns',0), now_ns, PRODUCTION_MAX_AGE_NS)

    fresh = []
    stale_count = 0
    for o in objects:
      if self._fresh(o, now_ns):
        fresh.append(o)
      else:
        stale_count += 1

    self._update_history(fresh, model_path, path_valid, now_s)

    candidates = []
    for src in fresh:
      o = dict(src)
      x = _finite(o.get('x'), math.nan)
      y = _finite(o.get('y'), math.nan)
      if not math.isfinite(x) or not math.isfinite(y) or not (0.5 <= x <= 140.0):
        continue

      key = self._identity(o)
      dpath = y - model_path_y_left(model_path, x) if path_valid else y
      vx = o.get('vx')
      vx = None if vx is None else _finite(vx)
      vlead = max(0.0, _finite(v_ego) + vx) if v_ego_valid and vx is not None else None
      motion = self._motion(key)
      ev = self._evidence(o)
      physical = o.get('source') != 'c4_camera'
      camera_prob = float(o.get('camera_prob',o.get('prob',0.0)) or 0.0)
      path_occ = abs(dpath) <= PATH_OVERLAP_HALF_M
      stationary = None if vlead is None else vlead <= STATIONARY_MAX_VLEAD_MPS
      age_ms = max(0.0,(int(now_ns)-int(o.get('recv_ns',now_ns)))/1e6)

      close = x <= 5.0
      min_span = CUTIN_CLOSE_CONFIRM_S if close else CUTIN_CONFIRM_S
      min_rate = CUTIN_CLOSE_MIN_INWARD_RATE_MPS if close else CUTIN_MIN_INWARD_RATE_MPS
      min_cons = CUTIN_CLOSE_MIN_CONSISTENCY if close else CUTIN_MIN_CONSISTENCY
      cutin_confirmed = (
        path_valid and physical and CUTIN_MIN_DREL_M <= x <= CUTIN_MAX_DREL_M
        and abs(dpath) <= CUTIN_NEAR_HALF_M
        and motion['span_s'] >= min_span
        and motion['inward_rate'] >= min_rate
        and motion['inward_disp'] >= CUTIN_MIN_INWARD_DISP_M
        and motion['consistency'] >= min_cons
      )

      cutin_score = 0.0
      if path_valid and physical and abs(dpath) <= CUTIN_NEAR_HALF_M and motion['inward_rate'] > 0:
        cutin_score = min(1.0,
          .30*min(1.0,motion['inward_rate']/.5)
          +.25*min(1.0,max(0.0,motion['inward_disp'])/.6)
          +.25*motion['consistency']
          +.20*(1.0 if ev['cross_sensor'] else .5)
        )

      cutout_score = 0.0
      if path_valid:
        cutout_score = min(1.0,
          .55*min(1.0,max(0.0,motion['outward_rate'])/.5)
          +.45*min(1.0,max(0.0,motion['outward_disp'])/.6)
        )

      stationary_supported = (
        path_valid and v_ego_valid and stationary is True and physical and path_occ
        and motion['span_s'] >= STATIONARY_CONFIRM_S
        and (ev['camera'] or ev['scc'] or bool(o.get('front_link')))
      )
      # Shadow-log review found occasional 100-120 m physical_in_path fallbacks
      # with no independent confirmation. Keep them visible as candidates but do
      # not promote them to leadOne until camera/SCC/corner-front evidence exists.
      far_unconfirmed = (
        physical and x > RADAR_ONLY_MAX_DREL_M
        and not (ev['camera'] or ev['scc'] or bool(o.get('front_link')))
      )
      moving_supported = (
        stationary is not True and path_occ and not far_unconfirmed
        and (physical or camera_prob >= CAMERA_ONLY_MIN_PROB)
        and (motion['span_s'] >= MOVING_CONFIRM_S or ev['cross_sensor'] or o.get('source')=='front_track')
      )
      camera_only_supported = (
        o.get('source')=='c4_camera' and path_occ and camera_prob >= CAMERA_ONLY_MIN_PROB
      )

      candidates.append({
        'obj':o,'key':key,'x':x,'y':y,'dpath':dpath,'vx':vx,'vlead':vlead,
        'motion':motion,'evidence':ev,'physical':physical,'stationary':stationary,
        'stationary_supported':stationary_supported,'moving_supported':moving_supported,
        'camera_only_supported':camera_only_supported,'path_occupied':path_occ,
        'cutin_confirmed':cutin_confirmed,'cutin_score':cutin_score,
        'cutout_score':cutout_score,'age_ms':age_ms,'far_unconfirmed':far_unconfirmed,
      })

    eligible = [c for c in candidates if c['path_occupied'] and
                (c['moving_supported'] or c['stationary_supported'] or c['camera_only_supported'])]

    l1c = next((c for c in eligible if c['key']==self.last_l1_key),None)
    l1_reidentified = False
    if l1c is None:
      l1c = self._reidentify(eligible,self.last_l1_state,now_s)
      if l1c is not None:
        l1c['reidentified']=True
        l1_reidentified=True
    if l1c is None:
      scored = []
      for c in eligible:
        ev = c['evidence']
        reliability = (3 if c['physical'] else 0)+(2 if ev['camera'] else 0)+(2 if ev['scc'] else 0)+(1 if ev['front'] else 0)
        scored.append((c['x']-1.5*reliability,c))
      l1c = min(scored,key=lambda z:z[0])[1] if scored else None
    self.last_l1_key = l1c['key'] if l1c else None
    if l1c is not None:
      self.last_l1_state={'x':l1c['x'],'y':l1c['y'],'vx':l1c.get('vx'),'t':now_s,'key':l1c['key']}

    lead1 = None
    if l1c:
      if l1c['stationary_supported']: reason='stationary_cross_sensor'
      elif l1c['obj'].get('source')=='c4_camera': reason='vision_only_fallback'
      elif l1c['evidence']['camera'] and l1c['evidence']['scc']: reason='radar_camera_scc'
      elif l1c['evidence']['camera']: reason='radar_camera'
      elif l1c['evidence']['scc']: reason='radar_scc'
      else: reason='physical_in_path'
      lead1 = self._lead_dict(l1c,'leadOne',reason,100.0-l1c['x'])

    duplicate_suppressed = 0
    cutins = []
    for c in candidates:
      if not c['cutin_confirmed']:
        continue
      if l1c is not None and c['key']==l1c['key']:
        continue
      if l1c is not None and self._same_physical(c,l1c):
        duplicate_suppressed += 1
        continue
      if l1c is not None and c['x'] >= l1c['x']-.5:
        continue
      cutins.append(c)

    l2c = None
    l2reason = None
    l2_reidentified = False
    if cutins:
      l2c = next((c for c in cutins if c['key']==self.last_l2_key),None)
      if l2c is None:
        l2c = self._reidentify(cutins,self.last_l2_state,now_s)
        if l2c is not None:
          l2c['reidentified']=True
          l2_reidentified=True
      if l2c is None:
        l2c = max(cutins,key=lambda c:(c['cutin_score'],-c['x']))
      l2reason = 'physical_dpath_cutin'

    if l2c is None and l1c is not None and l1c['stationary'] is False and l1c['cutout_score'] >= .55:
      moving_equiv = l1c['x']+(l1c['vlead']**2)/(2.0*2.5) if l1c['vlead'] is not None else l1c['x']
      shadows = []
      for c in candidates:
        if c['key']==l1c['key'] or not c['stationary_supported']:
          continue
        if self._same_physical(c,l1c):
          duplicate_suppressed += 1
          continue
        if c['x'] >= l1c['x']+STATIONARY_SHADOW_MIN_GAP_M and c['x'] < moving_equiv:
          shadows.append(c)
      if shadows:
        l2c = min(shadows,key=lambda c:c['x'])
        l2reason = 'stationary_shadow_behind_cutout'

    self.last_l2_key = l2c['key'] if l2c else None
    if l2c is not None:
      self.last_l2_state={'x':l2c['x'],'y':l2c['y'],'vx':l2c.get('vx'),'t':now_s,'key':l2c['key']}
    lead2 = self._lead_dict(l2c,'leadTwo',l2reason,80.0-l2c['x']) if l2c else None

    prod1 = production.get('leadOne',{'status':False}) if production_valid else {'status':False}
    prod2 = production.get('leadTwo',{'status':False}) if production_valid else {'status':False}
    prod_age = None if not production_valid else round((int(now_ns)-int(production['recv_ns']))/1e6,1)

    display = []
    for c in sorted(candidates,key=lambda x:x['x'])[:32]:
      display.append({
        'key':c['key'],'source_key':c['obj'].get('key'),'vehicle_key':c['obj'].get('vehicle_key'),'source':c['obj'].get('source'),'x':round(c['x'],3),'y':round(c['y'],3),
        'vx':None if c['vx'] is None else round(c['vx'],3),'d_path':round(c['dpath'],3),
        'v_lead':None if c['vlead'] is None else round(c['vlead'],3),'age_ms':round(c['age_ms'],1),
        'path_occupied':c['path_occupied'],'stationary':c['stationary'],
        'stationary_supported':c['stationary_supported'],'cutin_confirmed':c['cutin_confirmed'],
        'cutin_score':round(c['cutin_score'],3),'cutout_score':round(c['cutout_score'],3),
        'camera_confirmed':c['evidence']['camera'],'scc_teacher_confirmed':c['evidence']['scc'],
        'front_link':c['obj'].get('front_link'),
        'vehicle_member_count':int(c['obj'].get('vehicle_member_count',1) or 1),
        'vehicle_cluster_keys':c['obj'].get('vehicle_cluster_keys',[c['key']]),
        'far_unconfirmed':bool(c.get('far_unconfirmed',False)),
        'reidentified':bool(c.get('reidentified',False)),
        'shadow_role':'L1' if lead1 and lead1.get('key')==c['key'] else ('L2' if lead2 and lead2.get('key')==c['key'] else ''),
      })

    path_age = None if not model_path_recv_ns else round((int(now_ns)-int(model_path_recv_ns))/1e6,1)
    vego_age = None if not v_ego_recv_ns else round((int(now_ns)-int(v_ego_recv_ns))/1e6,1)

    return {
      'version':3,'mode':'shadow_only','control_connected':False,'publishes_radarState':False,'can_tx':False,
      'leadOne':lead1 or {'status':False,'validationState':'unavailable','role':'leadOne','controlConnected':False},
      'leadTwo':lead2 or {'status':False,'validationState':'unavailable','role':'leadTwo','controlConnected':False},
      'stockLeadOne':prod1,'stockLeadTwo':prod2,
      'comparison':{
        'production_valid':production_valid,'production_age_ms':prod_age,
        'leadOne_agrees_stock':self._agree(lead1,prod1),'leadTwo_agrees_stock':self._agree(lead2,prod2),
        'leadOne_dRel_error_m':None if not (lead1 and prod1.get('status')) else round(float(lead1['dRel'])-float(prod1['dRel']),3),
        'leadTwo_dRel_error_m':None if not (lead2 and prod2.get('status')) else round(float(lead2['dRel'])-float(prod2['dRel']),3),
        'leadOne_vRel_error_mps':None if not (lead1 and prod1.get('status') and lead1.get('vRel') is not None) else round(float(lead1['vRel'])-float(prod1['vRel']),3),
      },
      'candidates':display,
      'stats':{
        'fresh_object_count':len(fresh),'stale_object_count':stale_count,
        'candidate_count':len(candidates),'path_occupied_count':sum(1 for c in candidates if c['path_occupied']),
        'confirmed_cutin_count':sum(1 for c in candidates if c['cutin_confirmed']),
        'stationary_supported_count':sum(1 for c in candidates if c['stationary_supported']),
        'duplicate_suppressed_count':duplicate_suppressed,
        'far_unconfirmed_rejected_count':sum(1 for c in candidates if c.get('far_unconfirmed')),
        'leadOne_reidentified':l1_reidentified,'leadTwo_reidentified':l2_reidentified,
        'duplicate_gate_m':[DUP_DX_M,DUP_DY_M,DUP_DV_MPS],
        'path_valid':path_valid,'path_age_ms':path_age,
        'v_ego_valid':v_ego_valid,'v_ego_age_ms':vego_age,
      },
      'limitations':[
        'shadow only: never drives radarState/planner/CAN',
        'cut-in lateral rate derives from dPath position history',
        'stationary confirmation requires fresh path/vEgo plus cross-sensor support',
        'leadTwo is a cut-in/stationary-shadow candidate, not simply the second-nearest car',
        'unconfirmed radar-only objects beyond 100 m are not promoted to leadOne',
      ],
    }
