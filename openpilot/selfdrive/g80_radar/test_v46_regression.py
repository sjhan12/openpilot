"""Offline tests for exact shipped FG14, BSD adapter and UI priority inputs."""
import copy
import unittest
from types import SimpleNamespace
import test_v45_regression  # offline openpilot package namespace only
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator
from openpilot.selfdrive.g80_radar.target_lane_gate import TargetLaneGate
from openpilot.selfdrive.g80_radar.bsd_monitor import BsdMonitor

HEALTH={'radar_frames_fresh':True,'carstate_fresh':True}
CLEAR={'available':True,'fresh':True,'blocked':{'left':False,'right':False},'state':{'left':'OFF','right':'OFF'}}


def road():
  pts=lambda y:[{'x':0.0,'y':y},{'x':100.0,'y':y}]
  return {'valid':True,'fresh':True,'path':pts(0),'road_edges':[{'points':pts(10)},{'points':pts(-10)}],
          'lane_lines':[{'prob':1.,'points':pts(y)} for y in (-5.4,-1.8,1.8,5.4)]}


def obj(d=7.2,x=20,vd=0,projected=True,key='V1'):
  return dict(key=key,canonical_key=key,x=x,y=d,road_d=d,road_s=x,
              road_projection_valid=projected,vx=0,source_age_ms=0,
              canonical_track_duration_s=2.,kf_d_sigma=.15,kf_x_sigma=.1,
              kf_frenet_valid=True,kalman_valid=True,kf_s=x,kf_d=d,kf_s_dot=0,
              kf_d_dot=vd,kf_s_sigma=.1,recv_ns=10_000_000_000)


class V46Tests(unittest.TestCase):
  def test_bsd_immediate_and_delayed_release(self):
    m=BsdMonitor();m.supported=True;t=10_000_000_000
    cs=SimpleNamespace(leftBlindspot=True,rightBlindspot=False,canValid=True)
    m.update(cs,t,t);self.assertTrue(m.snapshot(t)['blocked']['left'])
    cs.leftBlindspot=False
    for dt in (100,200,300,400):m.update(cs,t+dt*1_000_000,t+dt*1_000_000)
    self.assertTrue(m.snapshot(t+400_000_000)['blocked']['left'])
    m.update(cs,t+500_000_000,t+500_000_000)
    self.assertFalse(m.snapshot(t+500_000_000)['blocked']['left'])

  def test_bsd_dropout_does_not_clear_on(self):
    m=BsdMonitor();t=10_000_000_000
    m.update(SimpleNamespace(leftBlindspot=False,rightBlindspot=True,canValid=True),t,t)
    s=m.snapshot(t+1_000_000_000)
    self.assertTrue(s['blocked']['right']);self.assertFalse(s['available'])

  def test_bsd_invalid_transport(self):
    m=BsdMonitor();m.supported=True;t=10_000_000_000
    m.update(SimpleNamespace(leftBlindspot=False,rightBlindspot=False,canValid=True),t,t-2_000_000_000)
    self.assertFalse(m.snapshot(t)['available'])

  def test_bsd_missing_fields_unknown(self):
    m=BsdMonitor();t=10_000_000_000;m.update(SimpleNamespace(canValid=True),t,t)
    self.assertEqual(m.snapshot(t)['state']['left'],'UNKNOWN')

  def test_bsd_unsupported_default_false_unknown(self):
    m=BsdMonitor();m.supported=False;t=10_000_000_000
    m.update(SimpleNamespace(leftBlindspot=False,rightBlindspot=False,canValid=True),t,t)
    self.assertFalse(m.snapshot(t)['available'])

  def warmed(self,d,vd=0,projected=True):
    g=TargetLaneGate()
    for i in range(5):
      t=10_000_000_000+i*100_000_000;o=obj(d,vd=vd,projected=projected);o['recv_ns']=t;g.update([o],t)
    return g,o,t

  def test_outer_lane_both_sides_excluded(self):
    for sign in (-1,1):
      g,o,t=self.warmed(sign*7.2)
      self.assertEqual(g.select([o],sign,t)[0],[])

  def test_predictor_spike_not_incoming(self):
    g,o,t=self.warmed(7.2,vd=-5)
    self.assertEqual(g.select([o],1,t)[0],[])

  def test_current_lane_body_overlap_kept(self):
    for d in (3.6,5.4,6.2):
      g,o,t=self.warmed(d);self.assertEqual(len(g.select([o],1,t)[0]),1)

  def test_rear_unprojected_kept(self):
    g,o,t=self.warmed(7.2,projected=False)
    self.assertEqual(len(g.select([o],1,t)[0]),1)

  def test_measured_incoming_kept(self):
    g=TargetLaneGate()
    for i in range(5):
      t=10_000_000_000+i*100_000_000;o=obj(7.8-i*.12,vd=-1.2);o['recv_ns']=t;g.update([o],t)
    picked,a=g.select([o],1,t)
    self.assertEqual(len(picked),1);self.assertEqual(a['objects'][0]['reason'],'measured_incoming_keep')

  def test_duplicate_measurement_not_history(self):
    g=TargetLaneGate();o=obj()
    for i in range(10):g.update([o],10_000_000_000+i*100_000_000)
    self.assertEqual(len(g.select([o],1,10_900_000_000)[0]),1)

  def test_empty_confirmed_lanes_green(self):
    e=FutureGapEvaluator()
    for i in range(15):
      r=e.update([],now_ns=10_000_000_000+i*100_000_000,road_model=road(),bsd=CLEAR,monitor_health=HEALTH)
    self.assertEqual(r['left']['decision']['label'],'SAFE')
    self.assertEqual(r['right']['decision']['label'],'SAFE')

  def test_bsd_overrides_turn_and_road_absent(self):
    e=FutureGapEvaluator();b=copy.deepcopy(CLEAR);b['blocked']['left']=True;b['state']['left']='ON'
    r=e.update([],v_ego=2,left_blinker=True,steering_angle_deg=45,now_ns=10_000_000_000,
               road_curve_direction='LEFT',road_model={},bsd=b,monitor_health=HEALTH)
    self.assertEqual(r['left']['decision']['label'],'DANGER · BSD')
    self.assertEqual(r['driver_intent']['phase'],'BSD_OVERRIDE')
    self.assertEqual(r['driver_intent']['maneuver_context'],'BSD')
    self.assertNotEqual(r['right']['decision']['label'],'DANGER · BSD')

  def test_bsd_overrides_commit_latch(self):
    e=FutureGapEvaluator();e.intent_hist.update(committed=True,latched_decision={'label':'SAFE'})
    sides=[{'decision':{'state':'SAFE_SHADOW'},'decision_raw':{'state':'SAFE_SHADOW'}} for _ in range(2)]
    intent={'label':'SAFE','phase':'COMMIT_HOLD'};b=copy.deepcopy(CLEAR);b['blocked']['right']=True
    e._monitor_overrides(*sides,intent,'right',b,HEALTH)
    self.assertEqual(intent['label'],'DANGER · BSD');self.assertFalse(e.intent_hist['committed'])

  def test_input_loss_not_green(self):
    for b,h in (({},HEALTH),(CLEAR,{})):
      e=FutureGapEvaluator()
      for i in range(15):
        r=e.update([],now_ns=10_000_000_000+i*100_000_000,road_model=road(),bsd=b,monitor_health=h)
      self.assertEqual(r['left']['decision']['label'],'CHECK DATA')

  def test_tracks_unchanged(self):
    g,o,t=self.warmed(7.2);before=copy.deepcopy(o);g.select([o],1,t)
    self.assertEqual(o,before)

  def test_full_pipeline_outer_lane_with_predictor_wrong_lane(self):
    e=FutureGapEvaluator()
    for i in range(20):
      t=10_000_000_000+i*100_000_000;o=obj(7.2,x=0,vd=-4);o['recv_ns']=t
      o.update(imm_valid=True,imm_s=0,imm_d=3.6,imm_s_dot=0,imm_d_dot=-4)
      r=e.update([o],now_ns=t,road_model=road(),bsd=CLEAR,monitor_health=HEALTH)
    self.assertEqual(r['left']['target_lane_filter']['excluded_count'],0)
    self.assertEqual(r['left']['decision']['label'],'CHECK ?')
    self.assertIn('measured_predictor_lane_disagreement',r['left']['decision']['reasons'])

  def test_logged_opposite_side_disagreement_not_red(self):
    e=FutureGapEvaluator()
    for i in range(15):
      t=10_000_000_000+i*100_000_000;o=obj(-2.475,x=.7);o['recv_ns']=t
      o.update(imm_valid=True,imm_s=.7,imm_d=2.43,imm_s_dot=0,imm_d_dot=0)
      r=e.update([o],now_ns=t,road_model=road(),bsd=CLEAR,monitor_health=HEALTH)
    self.assertEqual(r['left']['decision']['label'],'CHECK ?')
    b=copy.deepcopy(CLEAR);b['blocked']['left']=True
    r=e.update([o],now_ns=t+1,road_model=road(),bsd=b,monitor_health=HEALTH)
    self.assertEqual(r['left']['decision']['label'],'DANGER · BSD')

  def test_full_pipeline_adjacent_occupant_red(self):
    e=FutureGapEvaluator()
    for i in range(15):
      t=10_000_000_000+i*100_000_000;o=obj(3.6,x=0);o['recv_ns']=t
      r=e.update([o],now_ns=t,road_model=road(),bsd=CLEAR,monitor_health=HEALTH)
    self.assertEqual(r['left']['decision']['label'],'DANGER')

  def test_right_bsd_does_not_mask_left(self):
    b=copy.deepcopy(CLEAR);b['blocked']['right']=True;b['state']['right']='ON'
    e=FutureGapEvaluator()
    for i in range(15):
      r=e.update([],now_ns=10_000_000_000+i*100_000_000,road_model=road(),bsd=b,monitor_health=HEALTH)
    self.assertEqual(r['right']['decision']['label'],'DANGER · BSD')
    self.assertEqual(r['left']['decision']['label'],'SAFE')


if __name__=='__main__':unittest.main(verbosity=2)
