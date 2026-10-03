"""FG15 target-lane membership gate, using measured d history before prediction.

Do not discard display tracks. Exclude only from the requested-side evaluator.
No central-line/color recognition is claimed by this module.
"""
from collections import deque
import math


def number(v):
  try:
    f=float(v)
    return f if math.isfinite(f) else None
  except (ValueError,TypeError):
    return None


class TargetLaneGate:
  def __init__(self):
    self.history={}
    self.last_ns=0

  def update(self, objects, now_ns):
    if now_ns < self.last_ns:
      self.history.clear()
    self.last_ns=now_ns
    for o in objects:
      key=str(o.get('canonical_key') or o.get('vehicle_key') or o.get('key') or '')
      d=number(o.get('road_d'))
      if d is None:
        d=number(o.get('y'))
      stamp=int(o.get('recv_ns') or 0)
      age=number(o.get('source_age_ms'))
      if not key or d is None or age is None or age>350 or age<0:
        continue
      # Raw rear y is not curve-corrected. Never use it to prove an exclusion.
      trusted=bool(o.get('road_projection_valid'))
      h=self.history.get(key)
      if h is None or now_ns-h['last']>500_000_000 or h['trusted']!=trusted:
        h={'rows':deque(), 'last':now_ns, 'stamp':None, 'trusted':trusted}
        self.history[key]=h
      if stamp and stamp==h['stamp']:
        continue
      h['stamp']=stamp
      h['last']=now_ns
      h['rows'].append((now_ns,d))
      while h['rows'] and now_ns-h['rows'][0][0]>800_000_000:
        h['rows'].popleft()
    for key,h in list(self.history.items()):
      if now_ns-h['last']>1_500_000_000:
        del self.history[key]

  def select(self, objects, target_idx, now_ns):
    center=target_idx*3.6
    chosen=[]; audit=[]
    for o in objects:
      key=str(o.get('canonical_key') or o.get('vehicle_key') or o.get('key') or '')
      h=self.history.get(key)
      reason='unverified_geometry_keep'
      exclude=False
      disagreement=False
      if h and h['trusted'] and now_ns-h['last']<=350_000_000:
        rows=list(h['rows']); d=rows[-1][1]; offset=abs(d-center)
        predicted_d=number(o.get('imm_d')) if o.get('imm_valid') else number(o.get('kf_d')) if o.get('kalman_valid') and o.get('kf_frenet_valid') else None
        disagreement=bool(predicted_d is not None and abs(predicted_d-d)>1.8 and offset>1.8+1.05)
        # Passenger footprint + additional margin. A physically overlapping
        # boundary object remains relevant even with zero lateral velocity.
        sigma=number(o.get('kf_d_sigma'))
        margin=1.05+0.35+min(1.5,max(0.0,sigma if sigma is not None else 1.5))
        duration=(rows[-1][0]-rows[0][0])/1e9
        if offset<=1.8+margin:
          reason='target_lane_or_body_overlap'
        elif len(rows)<3 or duration<0.3:
          reason='outside_track_warming_keep'
        else:
          approach=abs(rows[0][1]-center)-offset
          rate=approach/duration
          recent=list(zip(rows,rows[1:]))
          toward=sum(abs(b[1]-center)<abs(a[1]-center)-0.01 for a,b in recent)
          eta=(offset-(1.8+margin))/rate if rate>0.15 else None
          measured_inward=bool(rate>0.25 and approach>0.12 and toward>=2)
          stable=bool(max(x[1] for x in rows)-min(x[1] for x in rows)<0.20 and abs(rate)<0.20)
          if measured_inward and eta is not None and eta<=5.0:
            reason='measured_incoming_keep'
          elif stable:
            # A derivative-only KF/IMM spike must not override a stable sequence
            # of fresh, projected positions outside the target footprint.
            exclude=True;reason='outside_lane_keeping_excluded'
          else:
            reason='outside_motion_uncertain_keep'
        if disagreement:
          exclude=False;reason='measured_predictor_lane_disagreement'
        audit.append(dict(key=key,reason=reason,excluded=exclude,d_m=round(d,3),
                          coordinate_disagreement=disagreement,predictor_d_m=predicted_d,
                          target_center_m=center,observed_span_s=round(duration,3)))
      else:
        audit.append(dict(key=key,reason=reason,excluded=False))
      if not exclude:
        chosen.append(o)
    return chosen,dict(target_lane_index=target_idx,objects=audit,
                       excluded_count=sum(x['excluded'] for x in audit),
                       centerline_semantics='NOT_AVAILABLE',
                       note='measured target-lane filter; rear unprojected/uncertain objects retained')
