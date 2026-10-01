"""Offline monitor regression tests; no cereal, vehicle, or CAN required."""
import gzip
import importlib
import pathlib
import sys
import tempfile
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parent
# Offline namespace only, so the exact shipped module imports can be exercised.
for name, path in [('openpilot', ROOT), ('openpilot.selfdrive', ROOT), ('openpilot.selfdrive.g80_radar', ROOT)]:
    if name not in sys.modules:
        m = types.ModuleType(name); m.__path__ = [str(path)]; sys.modules[name] = m
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator
from openpilot.selfdrive.g80_radar.shadow_leads import extract_model_path, model_path_y_left
from openpilot.selfdrive.g80_radar.road_geometry import extract_road_model, path_as_tuples
from shadow_log_reader import iter_records


def decision(**fields):
    o = dict(key='V1', tts_lateral_relevant=True, current_gap_m=60,
             side_overlap_entry_s=100, lateral_lane_entry_s=0,
             conflict_entry_s=None)
    o.update(fields)
    return FutureGapEvaluator._decision({'fg12_evidence': {'observations': [o]}})


class Regression(unittest.TestCase):
    def test_coordinate_contract(self):
        for sign in [-1, 1]:
            model = types.SimpleNamespace(position=types.SimpleNamespace(x=[0, 20, 40], y=[0, sign*2, sign*4]))
            normalized = path_as_tuples(extract_road_model(model, 100))
            self.assertEqual(extract_model_path(model), normalized)
            self.assertAlmostEqual(model_path_y_left(normalized, 30), -sign*3)
            self.assertAlmostEqual((-sign*3) - model_path_y_left(normalized, 30), 0)
    def test_far_one_axis_not_check(self):
        self.assertEqual(decision()['state'], 'SAFE_SHADOW')
    def test_near_watch_retained(self):
        self.assertEqual(decision(current_gap_m=4)['state'], 'CAUTION_SHADOW')
    def test_fast_closing_watch_retained(self):
        self.assertEqual(decision(side_overlap_entry_s=4)['state'], 'CAUTION_SHADOW')
    def test_danger_retained(self):
        self.assertEqual(decision(fg12_2d_confirmed=True, conflict_now=True)['state'], 'BLOCKED_SHADOW')
    def test_pending_retained(self):
        self.assertEqual(decision(tts_candidate=True)['state'], 'CAUTION_SHADOW')
    def test_model_near_retained(self):
        self.assertEqual(decision(confirmed_prediction=True, future_min_m=3, future_min_t_s=3)['state'], 'CAUTION_SHADOW')
    def test_lane_uncertain_retained(self):
        e=FutureGapEvaluator()
        d=e._apply_road_lane_gate(decision(), {'status':'UNCERTAIN'})
        self.assertEqual(d['label_override'], 'CHECK ROAD')
    def test_lane_absent_retained(self):
        d=FutureGapEvaluator()._apply_road_lane_gate(decision(), {'status':'ABSENT'})
        self.assertEqual(d['label_override'], 'NO LANE')
    def test_reader_formats(self):
        with tempfile.TemporaryDirectory() as td:
            p=pathlib.Path(td)/'a.jsonl.gz.part'; data=b'{"type":"sample","a":1}\n'
            for contents in [data, gzip.compress(data), gzip.compress(data)+gzip.compress(data)]:
                p.write_bytes(contents); report={}; rows=list(iter_records(p,report))
                self.assertTrue(report['complete']); self.assertGreaterEqual(len(rows),1)
            p.write_bytes(gzip.compress(data*100)[:-8]); report={}
            rows=list(iter_records(p,report)); self.assertTrue(rows); self.assertFalse(report['complete'])
            p.write_bytes(data+b'{"type":'); report={}
            self.assertEqual(len(list(iter_records(p,report))),1)
            self.assertEqual(report['incomplete_lines'],1)

if __name__ == '__main__': unittest.main(verbosity=2)
