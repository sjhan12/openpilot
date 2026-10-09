#!/usr/bin/env python3
from __future__ import annotations
import atexit
import json, os, socket, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from openpilot.cereal import messaging
from openpilot.selfdrive.g80_radar.decoder import CORNER_A,CORNER_B,FRONT_GROUP1,FR_CMR,decode_corner24,decode_front_group1_candidate,decode_fr_cmr_reference,decode_rear_teacher_1ea
from openpilot.selfdrive.g80_radar.tracker import TrackStore,filtered_objects,occupied_zones
from openpilot.selfdrive.g80_radar.corner_fusion import CornerFusionTracker
from openpilot.selfdrive.g80_radar.front_domain import build_front_objects,associate_corner_front
from openpilot.selfdrive.g80_radar.teacher_fusion import SCC_CONTROL_ADDR,DEFAULT_SCC_BUS,decode_scc_teacher,SccFrontTeacherMatcher,choose_scc_teacher
from openpilot.selfdrive.g80_radar.android_packet import build_render_packet, PROTOCOL_VERSION
from openpilot.selfdrive.g80_radar.camera_fusion import decode_model_leads,CameraRadarFusion,VehicleFootprintTracker
from openpilot.selfdrive.g80_radar.shadow_leads import ShadowLeadVerifier,snapshot_production_radar_state
from openpilot.selfdrive.g80_radar.cut_event_shadow import make_cut_event_shadow
from openpilot.selfdrive.g80_radar.shadow_logger import LOGGER_SERVICE_VERSION
from openpilot.selfdrive.g80_radar.front_standard_preview import StandardFrontPreview
from openpilot.selfdrive.g80_radar.road_geometry import extract_road_model,road_model_with_age,path_as_tuples,annotate_objects
from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION, BUILD_TAG, COORDINATE_X_ORIGIN, EGO_DISPLAY_LENGTH_M, EGO_DISPLAY_CENTER_X_M, OBJECT_X_ADJUSTMENT_M, FUTURE_GAP_API_VERSION as BUILD_FUTURE_GAP_API_VERSION, SIDE_VISION_API_VERSION as BUILD_SIDE_VISION_API_VERSION, FRONT_CORNER_VISION_API_VERSION as BUILD_FRONT_CORNER_VISION_API_VERSION
from openpilot.selfdrive.g80_radar.kalman_motion import KalmanMotionTracker, KALMAN_API_VERSION
from openpilot.selfdrive.g80_radar.canonical_tracker import Canonical360Tracker
from openpilot.selfdrive.g80_radar.imm_motion import ImmMotionTracker, IMM_API_VERSION
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator, FUTURE_GAP_API_VERSION
from openpilot.selfdrive.g80_radar.traffic_signal_probe import TrafficSignalProbe
from openpilot.selfdrive.g80_radar.my_case_correcter import MLCaseCollector
from openpilot.selfdrive.g80_radar.bsd_monitor import BsdMonitor
from openpilot.selfdrive.g80_radar.lane_change_shadow import build_lane_change_shadow, build_shadow_lead_interface
from openpilot.selfdrive.g80_radar.vasm_warning import VASMWarningEvaluator
from openpilot.selfdrive.g80_radar.web_views import diagnostic_raw_filtered, annotate_raw, selected_shadow_leads, make_stage_audit, validate_stage_contract

UDP_HOST=os.getenv('G80_RADAR_UDP_HOST','255.255.255.255')
UDP_PORT=int(os.getenv('G80_RADAR_UDP_PORT','28991'))
HTTP_HOST=os.getenv('G80_RADAR_HTTP_HOST','0.0.0.0')
HTTP_PORT=int(os.getenv('G80_RADAR_HTTP_PORT','28992'))
WEB_STATUS_PATH=Path(os.getenv('G80_RADAR_WEB_STATUS_PATH','/data/radar/g80_web_status.json'))
STATE_PATH=Path(os.getenv('G80_RADAR_STATE_PATH','/dev/shm/g80_radar.json'))
PUBLISH_HZ=float(os.getenv('G80_RADAR_PUBLISH_HZ','10'))
state_lock=threading.Lock()
browser_lock=threading.Lock()
browser_last_poll_ns=0
BROWSER_ACTIVE_NS=int(float(os.getenv('G80_BROWSER_ACTIVE_SEC','2.0'))*1e9)
DEBUG_STATE_HZ=max(0.0,float(os.getenv('G80_DEBUG_STATE_HZ','0')))
UI_STATE_HZ=max(2.0,float(os.getenv('G80_UI_STATE_HZ','8.0')))
CAN_DRAIN_GUARD_S=max(0.002,float(os.getenv('G80_CAN_DRAIN_GUARD_MS','8.0'))/1000.0)
SIDE_VISION_UDP_PORT=int(os.getenv('G80_SIDE_VISION_UDP_PORT','28993'))
SIDE_VISION_FRESH_NS=int(float(os.getenv('G80_SIDE_VISION_FRESH_SEC','1.6'))*1e9)
FRONT_CORNER_VISION_UDP_PORT=int(os.getenv('G80_FRONT_CORNER_VISION_UDP_PORT','28996'))
FRONT_CORNER_VISION_FRESH_NS=int(float(os.getenv('G80_FRONT_CORNER_VISION_FRESH_SEC','1.6'))*1e9)

class _DisabledShadowLogger:
  def status(self):
    return {'enabled':False,'hard_disabled':True,'service_version':BUILD_VERSION,'files_created':0,'records':0,'event_records':0,'last_write_ns':0,'path':None,'last_error':'','policy':'V52R4 ML-only: continuous shadow log permanently disabled'}

def _blank_side_vision(error='waiting_for_g80sidevision'):
  return {'api_version':BUILD_SIDE_VISION_API_VERSION,'source':'G80_V51_SIDE_VISION','fusion_mode':'SHADOW_ONLY','enabled':False,'model_valid':False,'config_loaded':False,'camera_connected':False,'fresh':False,'inference_fresh':False,'usable':False,'age_ms':None,'inference_age_ms':None,'left':{'active':False,'raw_confidence':0.0,'score':0.0},'right':{'active':False,'raw_confidence':0.0,'score':0.0},'last_error':error,'setup_url_port':28994}

def _drain_side_vision(sock,state,now_ns):
  newest=None
  if sock is not None:
    for _ in range(16):
      try:
        dat,_addr=sock.recvfrom(65535)
      except BlockingIOError:
        break
      except OSError:
        break
      try:
        obj=json.loads(dat.decode('utf-8'))
        if isinstance(obj,dict) and int(obj.get('api_version',0) or 0)==BUILD_SIDE_VISION_API_VERSION:
          newest=obj
      except Exception:
        continue
  if newest is not None:
    state=newest
  out=dict(state or _blank_side_vision())
  pkt_ns=int(out.get('packet_mono_ns',0) or 0)
  age_ns=(int(now_ns)-pkt_ns) if pkt_ns else None
  fresh=bool(age_ns is not None and -50_000_000 <= age_ns <= SIDE_VISION_FRESH_NS)
  out['age_ms']=None if age_ns is None else round(age_ns/1e6,1)
  out['fresh']=fresh
  inf_ns=int(out.get('last_inference_mono_ns',0) or 0)
  inf_age_ns=(int(now_ns)-inf_ns) if inf_ns else None
  try:
    expected_interval_s=max(0.15,float(out.get('inference_interval_s',1.0) or 1.0))
  except Exception:
    expected_interval_s=1.0
  inference_fresh_limit_ns=int(max(2.0, expected_interval_s*2.5+0.5)*1e9)
  inference_fresh=bool(inf_age_ns is not None and -50_000_000 <= inf_age_ns <= inference_fresh_limit_ns)
  out['inference_age_ms']=None if inf_age_ns is None else round(inf_age_ns/1e6,1)
  out['inference_fresh']=inference_fresh
  out['usable']=bool(fresh and inference_fresh and out.get('camera_connected') and out.get('enabled') and out.get('model_valid') and out.get('config_loaded'))
  for side in ('left','right'):
    sd=dict(out.get(side,{}) or {})
    sd.setdefault('active',False);sd.setdefault('raw_confidence',0.0);sd.setdefault('score',0.0)
    sample_ns=int(sd.get('last_inference_mono_ns') or 0)
    sample_age_ns=int(now_ns)-sample_ns if sample_ns else None
    sd['inference_age_ms']=None if sample_age_ns is None else round(sample_age_ns/1e6,1)
    sd['inference_fresh']=bool(sample_age_ns is not None and -50_000_000 <= sample_age_ns <= inference_fresh_limit_ns)
    sd['effective_active']=bool(out['usable'] and sd['inference_fresh'] and sd.get('active'))
    out[side]=sd
  return out


def _blank_front_corner_vision(error='waiting_for_g80frontcornervision'):
  return {'api_version':BUILD_FRONT_CORNER_VISION_API_VERSION,'source':'G80_V52_FRONT_CORNER_VISION','fusion_mode':'SHADOW_ONLY','enabled':False,'model_valid':False,'config_loaded':False,'camera_connected':False,'fresh':False,'inference_fresh':False,'usable':False,'age_ms':None,'inference_age_ms':None,'fl':{'active':False,'raw_confidence':0.0,'score':0.0},'fr':{'active':False,'raw_confidence':0.0,'score':0.0},'last_error':error,'setup_url_port':28995,'model_role':'temporary_reuse_of_side_vasm'}

def _drain_front_corner_vision(sock,state,now_ns):
  newest=None
  if sock is not None:
    for _ in range(16):
      try:
        dat,_addr=sock.recvfrom(65535)
      except BlockingIOError:
        break
      except OSError:
        break
      try:
        obj=json.loads(dat.decode('utf-8'))
        if isinstance(obj,dict) and int(obj.get('api_version',0) or 0)==BUILD_FRONT_CORNER_VISION_API_VERSION:
          newest=obj
      except Exception:
        continue
  if newest is not None:
    state=newest
  out=dict(state or _blank_front_corner_vision())
  pkt_ns=int(out.get('packet_mono_ns',0) or 0)
  age_ns=(int(now_ns)-pkt_ns) if pkt_ns else None
  fresh=bool(age_ns is not None and -50_000_000 <= age_ns <= FRONT_CORNER_VISION_FRESH_NS)
  out['age_ms']=None if age_ns is None else round(age_ns/1e6,1)
  out['fresh']=fresh
  inf_ns=int(out.get('last_inference_mono_ns',0) or 0)
  inf_age_ns=(int(now_ns)-inf_ns) if inf_ns else None
  try: expected_interval_s=max(0.15,float(out.get('inference_interval_s',0.7) or 0.7))
  except Exception: expected_interval_s=0.7
  inference_fresh_limit_ns=int(max(1.8, expected_interval_s*2.5+0.5)*1e9)
  inference_fresh=bool(inf_age_ns is not None and -50_000_000 <= inf_age_ns <= inference_fresh_limit_ns)
  out['inference_age_ms']=None if inf_age_ns is None else round(inf_age_ns/1e6,1)
  out['inference_fresh']=inference_fresh
  out['usable']=bool(fresh and inference_fresh and out.get('camera_connected') and out.get('enabled') and out.get('model_valid') and out.get('config_loaded'))
  if not isinstance(out.get('fl'),dict): out['fl']=dict(out.get('left',{}) or {})
  if not isinstance(out.get('fr'),dict): out['fr']=dict(out.get('right',{}) or {})
  for side in ('fl','fr'):
    sd=dict(out.get(side,{}) or {})
    sd.setdefault('active',False);sd.setdefault('raw_confidence',0.0);sd.setdefault('score',0.0)
    sample_ns=int(sd.get('last_inference_mono_ns') or 0)
    sample_age_ns=int(now_ns)-sample_ns if sample_ns else None
    sd['inference_age_ms']=None if sample_age_ns is None else round(sample_age_ns/1e6,1)
    sd['inference_fresh']=bool(sample_age_ns is not None and -50_000_000 <= sample_age_ns <= inference_fresh_limit_ns)
    sd['effective_active']=bool(out['usable'] and sd['inference_fresh'] and sd.get('active'))
    out[side]=sd
  return out

def mark_browser_active():
  global browser_last_poll_ns
  with browser_lock:
    browser_last_poll_ns=time.monotonic_ns()

def browser_is_active(now_ns=None):
  if now_ns is None: now_ns=time.monotonic_ns()
  with browser_lock:
    last=browser_last_poll_ns
  return last>0 and (now_ns-last)<=BROWSER_ACTIVE_NS

def _optional_sub_sock(name):
  try:
    return messaging.sub_sock(name,timeout=0,conflate=True)
  except Exception:
    return None

latest_state={'version':BUILD_VERSION,'sensor_fused_objects':[],'corner_fused_objects':[],'front_objects':[],'standard_front_preview':[],'filtered_objects':[],'raw_objects':[],'traffic_signal_probe':{}}
latest_state_json=json.dumps(latest_state,separators=(',',':')).encode()

HTML=r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no"><title>G80 V52R7 · Lead/CUT Shadow Lab</title>
<style>
:root{--bg:#07111a;--panel:#0c1d29;--line:#2b4859;--text:#e7f4fa;--muted:#91abba;--cyan:#28dae3;--navy:#236cff;--pink:#ff559e;--lime:#7fe16e;--orange:#ff982e;--gold:#f6c744;--red:#fb4967}
*{box-sizing:border-box}html,body{margin:0;height:100%;width:100%;background:var(--bg);color:var(--text);font-family:Arial,'Noto Sans KR',sans-serif;overflow:hidden}
body{display:flex;flex-direction:column;min-width:0}
header{flex:0 0 auto;display:flex;align-items:center;justify-content:space-between;padding:13px 17px 9px;gap:12px;background:#06121b;border-bottom:1px solid var(--line)}
.brand strong{font-size:19px;letter-spacing:.2px;color:white}.brand .sub{font-size:11px;letter-spacing:.5px;color:var(--cyan);font-weight:700;margin-top:4px}
.conn{font-size:12px;color:var(--muted);white-space:nowrap;font-weight:bold}.conn.good{color:#8eebac}.conn.bad{color:var(--red)}
.toolbar{display:flex;flex-direction:column;align-items:stretch;padding:8px 14px;gap:7px;background:#071822;border-bottom:1px solid var(--line);min-width:0}
.tabRail{display:flex;gap:7px;width:100%;overflow-x:auto;scrollbar-width:thin;min-width:0;white-space:nowrap}
.tabRail button,.control button{background:#102838;color:#b6d1df;border:1px solid #365465;border-radius:10px;padding:11px 15px;font-size:12px;font-weight:800;letter-spacing:.12px;cursor:pointer;flex:none}
.tabRail button.active{background:#1257aa;color:#fff;border-color:#3dc0ff;box-shadow:inset 0 0 0 1px #4c99f9,0 0 14px #085d7d5e}
.tabRail button .small{display:block;font-size:10px;color:inherit;opacity:.72;margin-top:3px}.control{display:flex;gap:5px;align-items:center;width:100%;justify-content:flex-end;overflow-x:auto;scrollbar-width:thin;white-space:nowrap}
.control button{padding:9px 12px;font-size:11px}.control button.active{color:#fff;background:#1b5578;border-color:#65c7ec}.control .warn{border-color:#57744a;color:#a7ed9b}
.stagebar{display:flex;align-items:baseline;gap:12px;padding:7px 16px;background:#0b202c;border-bottom:1px solid #264453;min-height:38px}
.stagebar strong{color:var(--cyan);font-size:15px;flex:none}.stagebar span{font-size:11.5px;color:#c1d6e1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.main{flex:1 1 auto;display:flex;min-height:0;min-width:0}.scene{position:relative;flex:1 1 auto;min-width:0;background:#081b24;overflow:hidden}
#radar{position:absolute;inset:0;width:100%;height:100%;display:block}
.stage-pill{position:absolute;top:8px;left:10px;z-index:4;pointer-events:none;background:#081922d9;border:1px solid #2f5466;border-radius:9px;padding:7px 10px;max-width:38%;min-width:150px}
.stage-pill strong{font-size:13px;color:#d4f9ff}.stage-pill .mini{font-size:11px;color:#abc7d4;margin-top:3px;line-height:1.4}
.signal{position:absolute;top:10px;left:50%;transform:translateX(-50%);width:min(530px,52vw);min-height:95px;display:flex;gap:14px;align-items:center;padding:10px 16px;border:2px solid #3f5c6c;border-radius:16px;background:#07141ce8;box-shadow:0 10px 25px #0007;z-index:9;pointer-events:none}
.signal.off{display:none}.lights{flex:none;display:flex;gap:5px;align-items:center;border:2px solid #60727a;border-radius:13px;background:#101920;padding:7px 8px}
.lamp{width:27px;height:27px;border-radius:50%;border:1px solid #3b4d54;background:#202a30;display:grid;place-items:center;font-weight:900;font-size:20px;color:#6d797c}.lamp.arrow{border-radius:7px;font-size:21px}
.lamp.red.on{background:#e63349;box-shadow:0 0 17px #f83c57}.lamp.yellow.on{background:#e8b827;box-shadow:0 0 17px #ffd834}.lamp.green.on{background:#24d75f;box-shadow:0 0 18px #34f472}.lamp.arrow.on{background:#38d27b;color:#0d2516;box-shadow:0 0 12px #57e595}
.signal-text{min-width:0;flex:1}.signal-text strong{display:block;font-size:18px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.signal-text span{display:block;font-size:11px;color:#abc5d1;margin-top:5px}.signal-text small{display:block;font-size:9px;color:#9cb5c3;margin-top:3px}
.arrow-wrap{position:absolute;top:46%;z-index:8;transform:translateY(-50%);display:flex;flex-direction:column;align-items:center;gap:5px;pointer-events:none}
.arrow-wrap.left{left:10px}.arrow-wrap.right{right:10px}.big-arrow{height:98px;width:65px;background:#62727c;clip-path:polygon(0 50%,100% 0,75% 50%,100% 100%);filter:drop-shadow(0 0 7px #000)}.right .big-arrow{transform:scaleX(-1)}
.big-arrow.safe{background:#2de178}.big-arrow.check{background:#efbd26}.big-arrow.danger{background:#f64058}.big-arrow.road{background:#71848f;opacity:.70}.big-arrow.turn{background:#358ffc}.big-arrow.off{background:#526773;opacity:.55}
.arrow-cam{min-width:104px;background:#071922e8;border:1px solid #42606f;border-radius:6px;padding:5px 8px;font-size:11px;font-weight:900;color:#a9c2cf;text-align:center;white-space:nowrap;box-shadow:0 2px 7px #0007}
.arrow-cam.clear{border-color:#258a9b;color:#7de8f4;background:#08232ae8}.arrow-cam.car{border-color:#ff6f91;color:#ffd0db;background:#351221e8;box-shadow:0 0 10px #ff436557}.arrow-cam.wait{border-color:#526875;color:#8fa5b0;background:#101c22e8}
.arrow-label{background:#0a1b23db;border-radius:5px;padding:4px 7px;font-size:11px;font-weight:900;color:#eefaff;text-align:center}
.bottom{position:absolute;bottom:7px;left:12px;background:#07141bd7;border:1px solid #32516a;border-radius:7px;font-size:9px;padding:5px 7px;color:#9bc4d3;pointer-events:none}
.panel{width:375px;flex:none;background:#0a1821;border-left:1px solid #365261;overflow-y:auto;min-height:0;scrollbar-width:thin;scrollbar-color:#37617b transparent}
.block{margin:11px;border:1px solid #2b4756;background:#0a1d28;border-radius:9px;padding:11px}
.block h3{margin:0 0 8px;font-size:13px;color:#bde7fa;letter-spacing:.2px}.desc{font-size:11px;line-height:1.48;color:#a8c2ce;margin:2px 0 7px}
.metric{display:flex;align-items:baseline;justify-content:space-between;gap:9px;padding:5px 0;border-bottom:1px solid #29404e;font-size:11px;color:#a4c0ca}.metric b{font-size:12px;color:#e5f4f8;font-weight:700;text-align:right;max-width:58%;overflow-wrap:anywhere}
.modes{display:grid;grid-template-columns:1fr 1fr;gap:5px}.modecount{border:1px solid #284453;border-radius:6px;padding:6px 7px;display:flex;justify-content:space-between;font-size:10.5px;color:#aec5d1}.modecount b{color:#fff}.modecount.chosen{border-color:#46b1ec;background:#144367}
.dot{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:4px;vertical-align:-1px}
.rows{display:flex;flex-direction:column;gap:4px}.row{border-bottom:1px solid #253b47;padding:5px 2px;display:grid;grid-template-columns:10px 1fr auto;align-items:start;column-gap:7px}
.row .label{font-size:12px;font-weight:800;color:#e5f2fa}.row .detail{font-size:10px;color:#a3c1ce;line-height:1.35;margin-top:3px;word-break:break-word}.row .value{font-size:11px;text-align:right;white-space:nowrap}
.badge{display:inline-block;padding:2px 4px;border:1px solid #31546a;border-radius:3px;font-size:9px;color:#aed8ea}.alert{color:#ff9c9c;font-size:10px}
footer{flex:none;border-top:1px solid #284658;background:#07131c;padding:3px 11px;font-size:9px;color:#7c9cad;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
@media(max-width:1150px){.panel{width:310px}.signal{width:min(490px,62vw)}.tabRail button{padding:9px 10px;font-size:11px}.control button{font-size:10.5px;padding:8px 9px}.toolbar{gap:5px}}
@media(max-width:770px){.panel{width:230px}.brand strong{font-size:15px}.brand .sub{font-size:9.5px}.stagebar span{max-width:65vw}.signal{left:53%;min-height:76px;width:62vw;gap:7px;padding:7px 10px}.lamp{height:18px;width:18px;font-size:13px}.signal-text strong{font-size:12px}.big-arrow{height:70px;width:41px}.arrow-cam{min-width:78px;font-size:9.5px;padding:3px 5px}}
</style></head>
<body>
<header><div class="brand"><strong>G80 5-RADAR <span style="color:#2fdbf6">V52R7</span> · STAGE LAB</strong><div class="sub">8 STAGES · FG15 · C4 ROAD WIDE + CABIN · NO UVC / NO YOLO</div></div><span class="conn" id="conn">● CONNECTING</span></header>
<div class="toolbar"><div class="tabRail" id="modes">
<button data-mode="raw">RAW<span class="small">DECODE</span></button>
<button data-mode="raw_filtered">RAW FILTERED<span class="small">VALIDITY ONLY</span></button>
<button data-mode="corner_fused">CORNER RADAR<span class="small">FOUR CORNERS</span></button>
<button data-mode="front_fused">FRONT FUSED<span class="small">FRONT + C4</span></button>
<button data-mode="all" class="active">360 + FRONT<span class="small">CANONICAL</span></button>
<button data-mode="std">STD<span class="small">RADARPOINT PREVIEW</span></button>
<button data-mode="l1l2">L1 + L2<span class="small">SELECTED SHADOW</span></button>
<button data-mode="cut">CUT-IN / OUT<span class="small">TOP CANDIDATES</span></button>
</div><div class="control"><button data-range="short">SHORT</button><button data-range="long">LONG</button><button data-range="wide">WIDE</button><button data-range="drive" class="active">3-LANE</button><button id="signalToggle" class="warn">SIGNAL ON</button><button id="camSetup">C4 CABIN ↗</button><button id="frontCamSetup">C4 WIDE ROAD ↗</button><button id="hud">HUD ↗</button><button id="fs">⛶</button></div></div>
<div class="stagebar"><strong id="stageName">360 + FRONT</strong><span id="stageDesc">Canonical 360: front and corner radar with camera corroboration, once per Vxxxx</span></div>
<div class="main"><div class="scene" id="scene"><canvas id="radar"></canvas><div class="stage-pill"><strong id="stageLabel">360 + FRONT · 0 objects</strong><div class="mini" id="stageNote">SOURCE MASK / TRACE KEY · SORT NEAREST FIRST</div></div>
<div class="signal" id="signalPanel"><div class="lights"><span class="lamp red" id="lr"></span><span class="lamp yellow" id="ly"></span><span class="lamp arrow" id="la">➜</span><span class="lamp green" id="lg"></span></div><div class="signal-text"><strong id="sigTitle">SIGNAL ? · E2E DIAGNOSTIC</strong><span id="sigMeta">Waiting for model...</span><small>실제 신호등 색 분류 아님 · 모니터 전용</small></div></div>
<div class="arrow-wrap left"><div class="arrow-cam wait" id="leftCam">SIDE-L CAM --</div><div class="arrow-cam wait" id="leftFrontCam">WIDE-FL CAM --</div><div class="big-arrow off" id="leftArrow"></div><div class="arrow-label" id="leftLabel">LEFT --</div></div><div class="arrow-wrap right"><div class="arrow-cam wait" id="rightCam">SIDE-R CAM --</div><div class="arrow-cam wait" id="rightFrontCam">WIDE-FR CAM --</div><div class="big-arrow off" id="rightArrow"></div><div class="arrow-label" id="rightLabel">RIGHT --</div></div>
<div class="bottom">SOURCE COLORS · FRONT <span style="color:#367dff">■</span> &nbsp; FL <span style="color:#1bdded">■</span> &nbsp; FR <span style="color:#ff61a8">■</span> &nbsp; RL <span style="color:#89e773">■</span> &nbsp; RR <span style="color:#ffa03d">■</span> · FL/FR/RL/RR = CORNER RADAR SECTOR · 360 = INITIAL SOURCE COLOR LOCK</div>
</div><aside class="panel"><section class="block"><h3>단계 선택 · STAGE CONTRACT</h3><div class="desc" id="stageMeaning">Only the actual canonical 360 objects are displayed.</div><div class="modes" id="stageCounts"></div><div class="desc" id="stageAudit" style="margin-top:8px"></div></section>
<section class="block"><h3>CUT-IN / CUT-OUT · TOP 3 + 2</h3><div class="desc">레이더/KF/IMM 기반 후보 점수 0~100 (통계적 확률 아님). 제어 미연결.</div><div class="rows" id="cutCandidates"></div></section>
<section class="block"><h3>ROAD / GAP / RUNTIME</h3><div id="telemetry"></div></section>
<section class="block"><h3 id="objTitle">STAGE OBJECTS · NEAREST</h3><div class="desc">현재 선택 메뉴의 객체만 목록에 표시합니다. 물체 위치의 색은 센서 소유권 확정이 아닌 관측 영역 추정일 수 있습니다.</div><div class="rows" id="objects"></div></section>
<section class="block"><h3>REFERENCE / SOURCE QUALITY</h3><div class="desc" id="reference"></div></section></aside></div>
<footer>NO CAN TX · NO RADARSTATE INJECTION · SHADOW MONITOR ONLY · 거리 및 상태 표시는 검증용이며 운전 판단 근거가 아닙니다.</footer>
<script>
'use strict';
const MODE_INFO={
 raw:['RAW · DECODE','CAN에서 디코딩한 유효 필드의 트랙 스냅샷. 안정성 필터·퓨전 이전. 그룹1 미검증 후보도 여기에만 표시.'],
 raw_filtered:['RAW FILTERED · VALIDITY','raw의 유효성·연속성 게이트 통과분. 서로 다른 레이더의 근접 검출을 합치지 않아 중복이 정상적으로 보일 수 있음.'],
 corner_fused:['CORNER RADAR FUSED · FOUR CORNERS','전좌·전우·후좌·후우 CORNER RADAR 융합 결과만 표시. FRONT radar 및 CAMERA-only 객체는 여기서 제외.'],
 front_fused:['FRONT FUSED · FRONT + C4','front_sensor_objects만: 전방 reference/radar + C4 확인 또는 CAMERA-only. 기존 FRONT ROI |y|≤5.8m, x≥-0.5m 적용.'],
 all:['360 + FRONT · CANONICAL','최종 360 Canonical Vxxxx 객체. 4코너 + 전방 레이더 + C4 확인을 한 번만 표시 (전방을 중복 추가하지 않음).'],
 std:['STD · RADARPOINT PREVIEW','front group1 미검증 트랙을 standard RadarPoint 형태로 시험하는 독립 후보 뷰. 확정 여부는 품질 분류일 뿐 실제 제어 출력 아님.'],
 l1l2:['L1 + L2 · SECOND FORWARD ONLY','L2: L1 앞쪽 실측 두 번째 전방 차량 (C4 leadsV3[1] + 레이더 매칭 또는 NEXT-FRONT). CUT-IN과 STOP HAZARD는 독립 관측. 제어 미연결.'],
 cut:['CUT-IN / CUT-OUT · RADAR SHADOW','상위 CUT-IN 3대와 CUT-OUT 2대만 표시. 점수는 KF/궤적 근거 지표이고 실제 발생 확률이 아님. 레이더/FG15/속도 제어는 변경하지 않음.']
};
const COLORS={FRONT:'#367dff',FL:'#1bdded',FR:'#ff61a8',RL:'#89e773',RR:'#ffa03d',CAMERA:'#b6e7ff',FRONT_CANDIDATE:'#f1cc4a',UNKNOWN:'#9baebb',L1:'#ffdf52',L2:'#ff954f',STD_CONFIRMED:'#4be282',STD_CORROBORATED:'#eccc51',STD_CANDIDATE:'#a7bac9'};
const KEYS={raw:'raw_objects',raw_filtered:'raw_filtered_objects',corner_fused:'corner_fused_objects',front_fused:'front_sensor_objects',all:'sensor_fused_objects',l1l2:'selected_shadow_leads',std:'standard_front_preview',cut:'cut_candidates'};
const LONG_RANGE={short:{rear:-15,front:35,step:5,stretch:false},long:{rear:-50,front:100,step:10,stretch:false},wide:{rear:-30,front:70,step:10,stretch:true,lat:12.6,wide:true},drive:{rear:-18,front:44,step:5,stretch:true,lat:7.2,drive:true,perspective:true}};
const qs=id=>document.getElementById(id),cvs=qs('radar'),scene=qs('scene'),ctx=cvs.getContext('2d'),canonColor=new Map();function getPref(k){try{return localStorage.getItem(k)}catch(_){return null}}
function setPref(k,v){try{localStorage.setItem(k,v)}catch(_){}}
let state=null,lastRx=0,mode='all',rangeMode='drive',signalOn=getPref('g80_signal')!=='0',visible=[];let cw=0,ch=0;
function dataList(key){return Array.isArray(state?.[key])?state[key]:[]}
function stageItems(){const key=KEYS[mode];return mode==='all'?canonicalOnly(dataList(key)):dataList(key)}
function canonicalOnly(src){const m=new Map();for(const o of src){const k=String(o.canonical_key||o.vehicle_key||o.key||'');if(!k)continue;if(!m.has(k)||rank(o)>rank(m.get(k)))m.set(k,o)}return Array.from(m.values())}
function rank(o){return ({CONFIRMED:4,MEASURED:3,PREDICTED:2,STALE:1}[String(o.display_state||'').toUpperCase()]||0)*10000-Math.min(9999,Number(o.source_age_ms??9999))}
function n(v,d=1){return Number.isFinite(Number(v))?Number(v).toFixed(d):'--'}
function xySector(o){const x=Number(o.x),y=Number(o.y);if(!Number.isFinite(x)||!Number.isFinite(y))return'UNKNOWN';if(Math.abs(y)<.8)return'UNKNOWN';return x>=0?(y>0?'FL':'FR'):(y>0?'RL':'RR')}
function mask(o){const v=o.source_mask||o.canonical_domains||o.trace_canonical_domains||[];return Array.isArray(v)?v.map(String).map(x=>x.toUpperCase()):[]}
function origin(o){const src=String(o.source||'').toLowerCase(),phys=String(o.stage_origin||'').toUpperCase();if(mode==='std'){const q=String(o.preview_quality||'').toUpperCase();return q.includes('CONFIRM')?'STD_CONFIRMED':q.includes('CORROB')?'STD_CORROBORATED':'STD_CANDIDATE'}
 if(mode==='l1l2')return o.shadow_role==='L1'?'L1':'L2';
 if(mode==='cut'){const p=String(o.canonical_primary_domain||'').toUpperCase();if(p==='FRONT')return 'FRONT';if(['FL','FR','RL','RR'].includes(p))return p;return xySector(o)}
 if(mode==='raw'||mode==='raw_filtered'){if(src==='front_group1_candidate')return'FRONT_CANDIDATE';if(phys==='FRONT_REFERENCE'||src==='fr_cmr_reference')return'FRONT';if(src==='corner24')return xySector(o);return'UNKNOWN'}
 if(mode==='corner_fused')return ['FL','FR','RL','RR'].includes(String(o.sector||''))?String(o.sector):xySector(o);
 if(mode==='front_fused')return o.camera_only||src==='c4_camera'?'CAMERA':'FRONT';
 // Canonical color is locked to Vxxxx, but source-mask always exposes all contributors.
 const k=String(o.canonical_key||o.vehicle_key||o.key||'');if(canonColor.has(k)){const prev=canonColor.get(k);prev.seen=Date.now();return prev.name;}
 let candidate='UNKNOWN';const dm=mask(o),primary=String(o.canonical_primary_domain||'').toUpperCase();if(src.includes('front')||src.includes('fr_cmr')||primary==='FRONT')candidate='FRONT';else if(src.includes('corner')||dm.some(x=>['FL','FR','RL','RR'].includes(x)))candidate=xySector(o);else if(o.camera_only||src==='c4_camera')candidate='CAMERA';else if(dm.includes('FRONT'))candidate='FRONT';
 if(!COLORS[candidate])candidate='UNKNOWN';if(k)canonColor.set(k,{name:candidate,seen:Date.now()});return candidate;
}
function col(o){return COLORS[origin(o)]||COLORS.UNKNOWN}
function traceLabel(o){if(mode==='raw'||mode==='raw_filtered')return String(o.key||'?');if(mode==='std')return String(o.track_id!=null?'RP'+o.track_id:(o.key||'?'));if(mode==='l1l2')return o.shadow_role+' '+String(o.canonical_key||o.key||'');if(mode==='cut')return (o.kind==='CUT-IN'?'IN ':'OUT ')+String(o.key||'?');return String(o.canonical_key||o.vehicle_key||o.key||'?')}
function resize(){const rect=scene.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2);cw=Math.max(1,rect.width);ch=Math.max(1,rect.height);cvs.width=Math.round(cw*dpr);cvs.height=Math.round(ch*dpr);cvs.style.width=cw+'px';cvs.style.height=ch+'px';ctx.setTransform(dpr,0,0,dpr,0,0)}
function viewCfg(){return LONG_RANGE[rangeMode]||LONG_RANGE.drive}
function mppY(){const a=viewCfg();return ch/(a.front-a.rear)}
function mppX(){const a=viewCfg();return a.stretch?cw/(2*a.lat):mppY()}
function persp(x){const a=viewCfg();if(!a.perspective)return 1;const f=Math.max(0,Number(x));return Math.max(.50,1/(1+f/55))}
function loc(x,y){return[cw*.5-Number(y)*mppX()*persp(x),(viewCfg().front-Number(x))*mppY()]}
function inView(o){const a=viewCfg(),x=Number(o.x),y=Number(o.y),half=a.stretch?a.lat:cw/(2*mppX());return Number.isFinite(x)&&Number.isFinite(y)&&x>=a.rear&&x<=a.front&&y>=-half/persp(x)&&y<=half/persp(x)}
function ptXY(p){if(Array.isArray(p))return[Number(p[0]),Number(p[1])];return[Number(p?.x),Number(p?.y)]}
function line(points,color,width=1,dash=[]){if(!Array.isArray(points)||points.length<2)return;ctx.beginPath();ctx.setLineDash(dash);ctx.strokeStyle=color;ctx.lineWidth=width;let once=false;for(const p of points){const [x,y]=ptXY(p);if(!Number.isFinite(x)||!Number.isFinite(y))continue;const at=loc(x,y);if(!once){ctx.moveTo(...at);once=true}else ctx.lineTo(...at)}if(once)ctx.stroke();ctx.setLineDash([])}
function roadYAtX(x){const rm=state?.road_model||{},p=rm.path||[];if(!rm.fresh||p.length<2)return null;const arr=p.map(ptXY).filter(z=>Number.isFinite(z[0])&&Number.isFinite(z[1])).sort((a,b)=>a[0]-b[0]);if(arr.length<2)return null;const xmax=Number(rm.path_x_max_m??arr[arr.length-1][0]),margin=Number(rm.path_projection_margin_m??4);if(x>xmax+margin)return null;let prev=arr[0];for(let i=1;i<arr.length;i++){const cur=arr[i];if(cur[0]>=x){const dx=cur[0]-prev[0];if(Math.abs(dx)<1e-6)return prev[1];const t=(x-prev[0])/dx;return prev[1]+(cur[1]-prev[1])*t}prev=cur}return arr[arr.length-1][1]}
function offsetPath(offset){const rm=state?.road_model||{},p=(rm.path||[]).map(ptXY).filter(z=>Number.isFinite(z[0])&&Number.isFinite(z[1]));if(p.length<2)return[];const out=[];for(let i=0;i<p.length;i++){const a=p[Math.max(0,i-1)],b=p[Math.min(p.length-1,i+1)],dx=b[0]-a[0],dy=b[1]-a[1],nn=Math.hypot(dx,dy)||1,nx=-dy/nn,ny=dx/nn;out.push({x:p[i][0]+nx*offset,y:p[i][1]+ny*offset})}return out}
function straightFallback(x0,x1,offset){return[{x:x0,y:offset},{x:x1,y:offset}]}
function drawRoad(){const a=viewCfg(),rm=state?.road_model||{},fresh=!!rm.fresh&&Array.isArray(rm.path)&&rm.path.length>1,laneW=3.6,outer=laneW*1.5;
 const g=ctx.createLinearGradient(0,0,0,ch);g.addColorStop(0,'#0a1d28');g.addColorStop(.55,'#07141c');g.addColorStop(1,'#030a0f');ctx.fillStyle=g;ctx.fillRect(0,0,cw,ch);
 ctx.font='10px Arial';ctx.textAlign='left';for(let xx=Math.ceil(a.rear/a.step)*a.step;xx<=a.front;xx+=a.step){const y=loc(xx,0)[1];ctx.strokeStyle=xx===0?'#77909c':'#17303c';ctx.lineWidth=xx===0?1.5:1;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(cw,y);ctx.stroke();ctx.fillStyle='#86a5b5';ctx.fillText(xx===0?'0m FRONT':((xx>0?'+':'')+xx+'m'),6,y-4)}
 const offsets=a.drive?[-outer,-laneW/2,laneW/2,outer]:[-9,-5.4,-1.8,1.8,5.4,9];
 if(fresh){
   let actual=(rm.lane_lines||[]).filter(ln=>Number(ln.prob||0)>=.35&&Array.isArray(ln.points)&&ln.points.length>1);
   let fillLeft=offsetPath(outer),fillRight=offsetPath(-outer);
   if(a.drive&&actual.length>=4){const meanY=ln=>{const pts=(ln.points||[]).map(ptXY);const n=Math.min(8,pts.length);if(!n)return 0;let z=0;for(let i=0;i<n;i++)z+=pts[i][1]||0;return z/n};const sorted=actual.slice().sort((u,v)=>meanY(v)-meanY(u));fillLeft=sorted[0].points||fillLeft;fillRight=sorted[sorted.length-1].points||fillRight}
   if(fillLeft.length>1&&fillRight.length>1){ctx.fillStyle=a.drive?'rgba(22,212,227,.050)':'rgba(22,212,227,.036)';ctx.beginPath();let started=false;for(const p of fillLeft){const [x,y]=ptXY(p),q=loc(x,y);if(!started){ctx.moveTo(...q);started=true}else ctx.lineTo(...q)}for(let i=fillRight.length-1;i>=0;i--){const [x,y]=ptXY(fillRight[i]);ctx.lineTo(...loc(x,y))}ctx.closePath();ctx.fill()}
   if(!a.drive||actual.length<3)for(const off of offsets)line(offsetPath(off),'rgba(92,119,133,.30)',1,[7,10]);
   for(const off of offsets)line(straightFallback(a.rear,0,off),'rgba(92,119,133,.30)',1,[7,10]);
   for(const e of rm.road_edges||[])line(e.points||[],'#75a1b0',1,[3,8]);
   let nActual=0;for(const ln of rm.lane_lines||[]){const pr=Number(ln.prob||0);if(pr<.20)continue;nActual++;ctx.globalAlpha=Math.max(.28,Math.min(1,.30+.70*pr));line(ln.points||[],'#b6dded',pr>.6?2.2:1.35,[10,8]);ctx.globalAlpha=1}
   line(rm.path||[],'#29dbe7',2.0,[]);
   if(a.drive){const labelX=Math.min(10,a.front*.30),base=roadYAtX(labelX)??0;ctx.textAlign='center';ctx.font='bold 10px Arial';ctx.fillStyle='#91afbe';for(const [lab,off] of [['L1',laneW],['EGO',0],['R1',-laneW]])ctx.fillText(lab,loc(labelX,base+off)[0],loc(labelX,base+off)[1]-4);ctx.textAlign='left'}
   ctx.fillStyle='rgba(183,245,255,.82)';ctx.font='bold 10px Arial';ctx.fillText(`C4 ${rm.curve_direction||'ROAD'} · lane ${nActual}/${rm.confident_lane_lines??0} · horizon ${rm.path_x_max_m==null?'--':Number(rm.path_x_max_m).toFixed(0)+'m'} · ${rm.age_ms==null?'--':Number(rm.age_ms).toFixed(0)+'ms'}`,10,18);
 }else{
   for(const off of offsets)line(straightFallback(a.rear,a.front,off),'rgba(92,119,133,.35)',1,[9,9]);
   ctx.fillStyle='rgba(255,107,107,.9)';ctx.font='bold 10px Arial';ctx.fillText('C4 ROAD MODEL STALE/UNAVAILABLE · straight fallback',10,18);
 }
}
function clampPx(v,lo,hi){return Math.max(lo,Math.min(hi,v))}
function carSymbol(x,y,color,label,o,ego=false){const a=viewCfg();let [sx,sy]=loc(x,y),width,height;
 // V51-H geometry review:
 // - x/y radar coordinates stay untouched.
 // - DRIVE/WIDE body length follows the longitudinal scale with only a small readability floor.
 // - width is derived from body length, not lateral mppX, because WIDE deliberately stretches the road laterally.
 //   This keeps the G80 visually long without making only the ego car abnormally wide.
 if(a.drive){const ps=persp(x);if(ego){height=clampPx(5.2*mppY(),48,66);width=clampPx(height/1.95,24,34)}else{height=clampPx(4.6*mppY()*ps,32,54);width=clampPx(height/1.90,17,29)}}
 else if(a.wide){if(ego){height=clampPx(5.2*mppY(),34,52);width=clampPx(height/2.15,18,26)}else{height=clampPx(4.6*mppY(),28,44);width=clampPx(height/2.00,14,24)}}
 else{height=Math.max(28,(ego?4.8:4.5)*mppY());width=Math.max(14,Math.min(1.8*mppX(),height/1.45))}if(ego)sy=loc(0,0)[1]+height/2;
 ctx.save();ctx.globalAlpha=o?.display_state==='STALE'?.36:o?.display_state==='PREDICTED'?.70:1;ctx.lineWidth=a.drive?2.0:1.6;ctx.strokeStyle=color;ctx.fillStyle=ego?'#eefaff17':color+'35';const left=sx-width/2,top=sy-height/2;ctx.beginPath();ctx.roundRect(left,top,width,height,Math.min(9,width*.28));ctx.fill();ctx.stroke();ctx.fillStyle='#263c49';ctx.strokeStyle='#76909e';ctx.lineWidth=.8;ctx.beginPath();ctx.roundRect(sx-width*.27,sy-height*.28,width*.54,height*.56,Math.min(5,width*.16));ctx.fill();ctx.stroke();ctx.fillStyle=ego?'#e1faff':color;ctx.fillRect(sx-width*.40,sy-height*.44,width*.80,Math.max(2,height*.06));ctx.globalAlpha=1;
 if(!ego){ctx.font=a.drive?'bold 11px Arial':'bold 10px Arial';ctx.textAlign='left';ctx.fillStyle=color;ctx.fillText(label||'?',sx+width*.62,sy-3);ctx.font='9px Arial';ctx.fillText(`${n(x)}m${o?.vx==null?'':' '+n(o.vx)+'m/s'}`,sx+width*.62,sy+10)}
 ctx.restore();return[sx,sy,Math.max(width,height)]}
function draw(){resize();drawRoad();carSymbol(0,0,'#effaff','G80',null,true);const sorted=visible.slice().sort((a,b)=>Math.abs(Number(a.x))-Math.abs(Number(b.x)));for(const o of sorted){if(!inView(o))continue;const color=col(o),id=traceLabel(o),role=String(o.shadow_role||''),label=mode==='l1l2'?role:mode==='raw'||mode==='raw_filtered'?String(o.key||'?'):id;const [sx,sy,size]=carSymbol(Number(o.x),Number(o.y),color,label,o);if(mode==='l1l2'){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.arc(sx,sy,size*.56,0,2*Math.PI);ctx.stroke()}if(mode==='std'){ctx.fillStyle=color;ctx.font='9px Arial';ctx.fillText(String(o.preview_quality||'CANDIDATE').toUpperCase(),sx+size*.40,sy+25)}if(mode==='cut'){ctx.strokeStyle=o.kind==='CUT-IN'?'#f6c744':'#ff8795';ctx.lineWidth=3;ctx.beginPath();ctx.arc(sx,sy,size*.64,0,2*Math.PI);ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.font='bold 11px Arial';ctx.fillText((o.kind==='CUT-IN'?'IN ':'OUT ')+o.score_index+'/100',sx-10,sy-size*.65)}}}
function classifyArrow(label){const t=String(label||'');if(t.startsWith('SAFE'))return'safe';if(t.startsWith('DANGER'))return'danger';if(t.startsWith('TURN'))return'turn';if(t.startsWith('ROAD ?')||t.startsWith('CHECK ROAD')||t.startsWith('NO LANE'))return'road';if(t.startsWith('CHECK'))return'check';return'off'}
function updateArrows(){const fg=state?.future_gap||{},di=fg.driver_intent||{},sv=state?.side_vision||{},fc=state?.front_corner_vision||{};

 const waitText=v=>!v.model_valid?'MODEL WAIT':!v.config_loaded?'CONFIG WAIT':!v.camera_connected?'CAM WAIT':!v.inference_fresh?'INFER WAIT':'CAM WAIT';
 for(const side of ['left','right']){let label=state?.vasm_warning?.[side]?.warning_label||fg[side]?.decision?.label||'--';if(di.active&&di.side===side&&(di.maneuver_context==='TURN'||di.committed))label=di.label||label;const type=classifyArrow(label);qs(side+'Arrow').className='big-arrow '+type;qs(side+'Label').textContent=side.toUpperCase()+' '+label+(state?.vasm_warning?.[side]?.camera_caution?' · CAM':'');
  const cam=qs(side+'Cam'),cs=sv[side]||{},score=Number(cs.score);if(sv.usable&&Number.isFinite(score)){const car=!!cs.effective_active;cam.className='arrow-cam '+(car?'car':'clear');cam.textContent=(side==='left'?'SIDE-L CAM ':'SIDE-R CAM ')+(car?'CAR ':'CLR ')+score.toFixed(2)}else{cam.className='arrow-cam wait';cam.textContent=(side==='left'?'SIDE-L CAM ':'SIDE-R CAM ')+waitText(sv)}
  const fcam=qs(side+'FrontCam'),key=side==='left'?'fl':'fr',fs=fc[key]||{},fscore=Number(fs.score);if(fc.usable&&Number.isFinite(fscore)){const car=!!fs.effective_active;fcam.className='arrow-cam '+(car?'car':'clear');fcam.textContent=(side==='left'?'WIDE-FL CAM ':'WIDE-FR CAM ')+(car?'CAR ':'CLR ')+fscore.toFixed(2)}else{fcam.className='arrow-cam wait';fcam.textContent=(side==='left'?'WIDE-FL CAM ':'WIDE-FR CAM ')+waitText(fc)}}
 const laneText=side=>{const a=fg[side]?.lane_availability||{};return a.reason==='path_too_short_for_lane_geometry'?'SHORT PATH':a.status||'UNKNOWN'};
 const rows=[['BSD L / R',(fg.bsd?.state?.left||'UNKNOWN')+' / '+(fg.bsd?.state?.right||'UNKNOWN')],['BSD input',fg.bsd?.available?'FRESH / SUPPORTED':'UNCONFIRMED'],['Lane geometry L/R',laneText('left')+' / '+laneText('right')],['Outer excluded L/R',String(fg.left?.target_lane_filter?.excluded_count||0)+' / '+String(fg.right?.target_lane_filter?.excluded_count||0)],['Central line','NOT CLASSIFIED']];
 for(const [label,value] of rows){const row=document.createElement('div');row.className='metric';const a=document.createElement('span'),b=document.createElement('b');a.textContent=label;b.textContent=value;row.append(a,b);qs('telemetry').append(row)}
}
function updateSignal(){qs('signalPanel').classList.toggle('off',!signalOn);qs('signalToggle').textContent=signalOn?'SIGNAL ON':'SIGNAL OFF';if(!state)return;const t=state.traffic_signal_probe||{},st=String(t.state||'UNKNOWN'),turn=String(t.turn_direction||'NONE');for(const id of ['lr','ly','la','lg'])qs(id).classList.remove('on');let text='SIGNAL UNKNOWN',color='#d7edf5';if(st==='GREEN_GO'){qs('lg').classList.add('on');text='GREEN / GO · E2E';color='#66eda0'}else if(st==='RED_CANDIDATE'){qs('lr').classList.add('on');text='RED ? · INFERRED STOP';color='#ff6673'}else if(st.includes('STOP')||st==='WATCH'){text=String(t.label||'STOP/HOLD · LIGHT UNKNOWN');color='#d7edf5'}else if(st==='WAIT_LEAD'){text='LEAD AHEAD · LIGHT UNKNOWN';}else if(st==='DRIVING'){text='DRIVING · LIGHT ?'}
 if(st==='GREEN_GO'&&['LEFT','RIGHT'].includes(turn)){qs('la').classList.add('on');qs('la').textContent=turn==='LEFT'?'←':'→'}else qs('la').textContent='↔';qs('sigTitle').textContent=text;qs('sigTitle').style.color=color;qs('sigMeta').textContent=`path ${n(t.path_horizon_m)}m · stop ${t.should_stop===true?'1':t.should_stop===false?'0':'?'} · SP green ${t.sunnypilot_green_alert?'1':'0'} · plan ${turn}`;
}
function renderCutCandidates(){const ev=state?.cut_event_shadow||{},dst=qs('cutCandidates'),arr=Array.isArray(ev.candidates)?ev.candidates.slice(0,5):[];dst.replaceChildren();if(!ev.path_valid){dst.textContent='C4 경로 정보 미확인 · CUT 후보 표시 중단';return}if(!arr.length){dst.textContent='높은 점수의 CUT-IN/OUT 후보 없음';return}for(const c of arr){const row=document.createElement('div');row.className='row';const dot=document.createElement('span');dot.className='dot';dot.style.background=c.kind==='CUT-IN'?'#f6c744':'#ff8795';const b=document.createElement('div'),t=document.createElement('div'),detail=document.createElement('div'),num=document.createElement('span');t.className='label';detail.className='detail';num.className='value';t.textContent=(c.kind==='CUT-IN'?'IN':'OUT')+' · '+String(c.key||'--');detail.textContent=String(c.status||'')+' / '+String(c.reason||'')+' / '+n(c.x)+'m · '+n(c.y)+'m / '+(c.ttlc_s==null?'TTLC --':'TTLC '+n(c.ttlc_s)+'s');num.textContent=String(c.score_index??'--')+'/100';b.append(t,detail);row.append(dot,b,num);dst.append(row)}}
function updateUI(){const d=MODE_INFO[mode],audit=state.web_stage_stats||{},cnt=audit.counts||{},fg=state.future_gap||{},di=fg.driver_intent||{},sig=state.traffic_signal_probe||{},rm=state.road_model||{},performance=state.performance_stats||{},sv=state.side_vision||{},fc=state.front_corner_vision||{};
 qs('stageName').textContent=d[0];qs('stageDesc').textContent=d[1];qs('stageMeaning').textContent=d[1];qs('stageLabel').textContent=d[0]+' · '+visible.length+' objects';qs('stageNote').textContent= mode==='all'?'ONE V-ID PER OBJECT · FRONT ALREADY INCLUDED':mode==='raw_filtered'?'NO CROSS-SOURCE DEDUP · RAW VALIDITY':'MONITOR ONLY · SENSOR PROVENANCE';
 const modes=['raw','raw_filtered','corner_fused','front_fused','all','std','l1l2','cut'];qs('stageCounts').innerHTML=modes.map(k=>`<div class="modecount ${k===mode?'chosen':''}"><span>${k==='raw_filtered'?'RAW FILT':k==='corner_fused'?'CORNER':k==='front_fused'?'FRONT':k==='all'?'360+FRONT':k==='l1l2'?'L1/L2':k==='cut'?'CUT':k.toUpperCase()}</span><b>${Number(cnt[k]??dataList(KEYS[k]).length)}</b></div>`).join('');
 const mismatches=state.web_stage_errors||[],miss=Number(audit.front_unmatched_inside_view_roi||0),oor=Number(audit.front_outside_view_roi||0);qs('stageAudit').innerHTML=`<b>RAW → FILTER:</b> ${cnt.raw??0} → ${cnt.raw_filtered??0} (실제 융합 입력 ${cnt.production_filtered??0})<br><b>FRONT ROI:</b> 캐노니컬 out ${oor}, 내부 trace 미연결 ${miss}<br><b>Canonical key 중복:</b> ${audit.canonical_duplicate_keys??0} · <b>계약:</b> <span style="color:${mismatches.length?'#fa8192':'#80e7a3'}">${mismatches.length?mismatches.join(', '):'OK'}</span>`;
 const left=fg.left?.decision||{},right=fg.right?.decision||{},s=fg.stats||{},fmt=v=>v==null?'--':n(v)+'m';const le=fg.left?.fg11_evidence||fg.left?.fg10_evidence||fg.left?.fg9_evidence||{},re=fg.right?.fg11_evidence||fg.right?.fg10_evidence||fg.right?.fg9_evidence||{};
 qs('telemetry').innerHTML=[['서버 버전',String(state.runtime_versions?.tag||'--').split('-').slice(0,2).join('-')],['FG/WEB',String(state.runtime_versions?.future_gap_api||'?')+' / '+String(state.version||'?')],['현재 모드',mode.toUpperCase()],['ROAD',rm.fresh?'FRESH '+n(rm.age_ms,0)+'ms':'STALE'],['Driver intent',String(di.state||'STANDBY')],['LEFT / RIGHT',(left.label||'--')+' / '+(right.label||'--')],['SIDE CAM L/R',sv.usable?((sv.left?.effective_active?'CAR ':'clear ')+n(sv.left?.score,2)+' / '+(sv.right?.effective_active?'CAR ':'clear ')+n(sv.right?.score,2)):'-- / --'],['SIDE CAM',sv.usable?('READY '+n(sv.inference_age_ms??sv.age_ms,0)+'ms · SHADOW'):(!sv.model_valid?'MODEL WAIT':(!sv.config_loaded?'CONFIG WAIT':(!sv.camera_connected?'CAMERA WAIT':(sv.fresh?'STATUS FRESH / INFERENCE WAIT':'STALE / WAIT'))))],['WIDE CAM FL/FR',fc.usable?((fc.fl?.effective_active?'CAR ':'clear ')+n(fc.fl?.score,2)+' / '+(fc.fr?.effective_active?'CAR ':'clear ')+n(fc.fr?.score,2)):'-- / --'],['WIDE CAM STATUS',fc.usable?('READY '+n(fc.inference_age_ms??fc.age_ms,0)+'ms · SHADOW'):(!fc.model_valid?'MODEL WAIT':(!fc.config_loaded?'CONFIG WAIT':(!fc.camera_connected?'CAMERA WAIT':(fc.fresh?'STATUS FRESH / INFERENCE WAIT':'STALE / WAIT'))))],['V53R1 WARN L/R',String(state.vasm_warning?.left?.warning_label||'WAIT')+' / '+String(state.vasm_warning?.right?.warning_label||'WAIT')],['WARN EVIDENCE L/R',String(state.vasm_warning?.left?.display_reason||'NONE')+' / '+String(state.vasm_warning?.right?.display_reason||'NONE')],['FG15 RAW L/R',String(state.vasm_warning?.left?.radar_label||'WAIT')+' / '+String(state.vasm_warning?.right?.radar_label||'WAIT')],['LANE GATE L/R',String(state.vasm_warning?.left?.road_gate||'UNKNOWN')+' / '+String(state.vasm_warning?.right?.road_gate||'UNKNOWN')],['SHADOW CAM L/R',String(state.lane_change_shadow?.left?.shadow_note||'WAIT')+' / '+String(state.lane_change_shadow?.right?.shadow_note||'WAIT')],['L1 / L2 shadow',(state.shadow_lead_interface?.leadOne?.candidate_valid?'L1 YES':'L1 --')+' / '+(state.shadow_lead_interface?.leadTwo?.candidate_valid?'L2 YES':'L2 --')],['L2 SELECT',String(state.shadow_leads?.leadTwo?.selectionMode||'NONE')],['CUT-IN TARGET',state.shadow_leads?.cutInLead?.status?(String(state.shadow_leads.cutInLead.key||'?')+' / '+n(state.shadow_leads.cutInLead.dRel,1)+'m'):'NONE'],['STOP HAZARD',state.shadow_leads?.stopHazard?.status?(String(state.shadow_leads.stopHazard.key||'?')+' / '+n(state.shadow_leads.stopHazard.dRel,1)+'m'):'NONE'],['L2 WHY',String(state.shadow_leads?.stats?.lead2_selection_reason||'WAIT')],['NEXT RADAR',state.shadow_leads?.nextForward?.status?('YES '+n(state.shadow_leads.nextForward.dRel,1)+'m'):'NONE'],['C4 model lead2',state.shadow_leads?.visionLeadTwo?.status?('YES '+n(state.shadow_leads.visionLeadTwo.dRel,1)+'m / '+(state.shadow_leads.visionLeadTwo.matchedRadar?'RADAR MATCH':'NO RADAR MATCH')):'NONE'],['CUT IN / OUT',String(state.cut_event_shadow?.cutin?.length??0)+' / '+String(state.cut_event_shadow?.cutout?.length??0)],['LEFT reason',(left.reasons||[]).join(', ')||'--'],['RIGHT reason',(right.reasons||[]).join(', ')||'--'],['NOW rear L/R',fmt(le.current_rear_gap_m)+' / '+fmt(re.current_rear_gap_m)],['2D CONFLICT L/R',fmt((fg.left?.fg12_evidence||fg.left?.fg11_evidence||{}).min_2d_conflict_time_s)+' / '+fmt((fg.right?.fg12_evidence||fg.right?.fg11_evidence||{}).min_2d_conflict_time_s)],['PRED rear L/R',fmt(le.predicted_rear_min_m)+' / '+fmt(re.predicted_rear_min_m)],['Core cycle',n(performance.processing_ms)+' ms'],['ML collector',n(performance.stage_ms?.ml_collector)+' ms'],['UI JSON',n(performance.ui_json_ms)+' ms'],['CAN drain',n(performance.can_drain_ms)+' ms'],['Publish',n(performance.publish_interval_ms)+' ms'],['PUB late',n(performance.publish_late_ms)+' ms'],['Shadow log',state.shadow_logger?.enabled?'LOGGING':'OFF']].map(([a,b])=>`<div class="metric"><span>${a}</span><b>${b}</b></div>`).join('');
 renderCutCandidates();qs('objTitle').textContent=d[0]+(mode==='cut'?' · RANKED':' · NEAREST');const rows=(mode==='cut'?visible.slice():visible.slice().sort((a,b)=>Math.abs(Number(a.x))-Math.abs(Number(b.x)))).slice(0,35);qs('objects').innerHTML=rows.length?rows.map(o=>{const color=col(o),p=origin(o),local=o.trace_local_key||o.key||'',c=String(o.canonical_key||''),badge=mode==='raw'||mode==='raw_filtered'?`${String(o.stage_origin||o.sensor||'')} · ${local}`:mode==='std'?String(o.preview_quality||'CANDIDATE'):mode==='l1l2'?String(o.shadow_validation_state||'SHADOW'):mode==='cut'?(String(o.status||'CANDIDATE')+' · '+String(o.reason||'')):mask(o).join('+')||p;
 return `<div class="row"><span class="dot" style="background:${color}"></span><div><span class="label">${traceLabel(o).replaceAll('<','&lt;')}</span><div class="detail">${badge.replaceAll('<','&lt;')} ${mode!=='raw'&&mode!=='raw_filtered'&&o.canonical_key?` · ${String(o.trace_match_method||'')}`:''}</div></div><span class="value">${n(o.x)}m<br>${n(o.y)}y<br>${mode==='cut'?(String(o.score_index)+'/100 · '+(o.ttlc_s==null?'TTLC --':'TTLC '+n(o.ttlc_s)+'s')):(o.vx==null?'--':n(o.vx)+'m/s')}</span></div>`}).join(''):'<span class="desc">이 단계에서 표시할 객체가 없습니다.</span>';
 qs('reference').innerHTML=`<b style="color:#367dff">FRONT</b> · <b style="color:#1bdded">FL</b> · <b style="color:#ff61a8">FR</b> · <b style="color:#89e773">RL</b> · <b style="color:#ffa03d">RR</b><br>코너 색은 위치상의 사분면을 근거로 합니다. A/B 실제 센서 소유권이 4개 코너별로 확정됐다는 뜻은 아닙니다.<br><br>front fused는 abs(y)≤5.8m의 기존 내부 ROI로 제한됩니다. RAW FILTERED는 그룹1 미검증 후보를 제외하며 cross-sensor merge 전 단계를 관찰합니다.<br><br>STD 품질 및 E2E 신호 상태는 제어 검증용 표시입니다.`;
 updateArrows();updateSignal();
}
function setMode(k){if(!MODE_INFO[k])return;mode=k;document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===k));if(state){visible=stageItems();updateUI();draw()}}
function setRange(k){if(!LONG_RANGE[k])return;rangeMode=k;document.querySelectorAll('[data-range]').forEach(b=>b.classList.toggle('active',b.dataset.range===k));if(state)draw()}
document.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>setMode(b.dataset.mode)));
document.querySelectorAll('[data-range]').forEach(b=>b.addEventListener('click',()=>setRange(b.dataset.range)));
qs('signalToggle').onclick=()=>{signalOn=!signalOn;setPref('g80_signal',signalOn?'1':'0');updateSignal()};qs('camSetup').onclick=()=>window.open('http://'+location.hostname+':'+String(state?.side_vision?.setup_url_port||28994)+'/','_blank');qs('frontCamSetup').onclick=()=>window.open('http://'+location.hostname+':'+String(state?.front_corner_vision?.setup_url_port||28995)+'/','_blank');qs('hud').onclick=()=>window.open('/hud','_blank');qs('fs').onclick=async()=>{try{document.fullscreenElement?await document.exitFullscreen():await document.documentElement.requestFullscreen()}catch(_){}};
let failures=0;async function poll(){try{const r=await fetch('/state',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);state=await r.json();lastRx=Date.now();failures=0;const rt=state.runtime_versions||{},bad=!!state.runtime_mismatch;qs('conn').textContent=`● LIVE V${state.version||'?'} FG${rt.future_gap_api||'?'} · ${n(state.performance_stats?.publish_interval_ms,0)}ms${bad?' · RUNTIME MISMATCH':''}`;qs('conn').className='conn '+(bad?'bad':'good');visible=stageItems();updateUI();draw();for(const[k,v]of canonColor){if(Date.now()-v.seen>15000)canonColor.delete(k)}}catch(e){failures++;qs('conn').textContent='● WEB /STATE UNAVAILABLE '+failures;qs('conn').className='conn bad';for(const side of ['left','right']){qs(side+'Arrow').className='big-arrow off';qs(side+'Label').textContent=side.toUpperCase()+' CHECK DATA'}}setTimeout(poll,200)}
window.addEventListener('resize',()=>{if(state)draw()});updateSignal();poll();
</script></body></html>
'''

HUD_HTML=r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover"><title>G80 V53R1 Driver HUD</title><style>
html,body{margin:0;width:100%;height:100%;background:#05090c;color:#fff;font-family:Arial,"Noto Sans KR",sans-serif;overflow:hidden}.wrap{height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center}.card{position:relative;width:96vw;height:82vh;border:4px solid #34454f;border-radius:28px;background:#0b1216;display:flex;align-items:center;justify-content:center;overflow:hidden}.center{z-index:3;min-width:40vw;max-width:52vw;text-align:center;padding:3vh 2vw;border-radius:22px;background:rgba(8,15,19,.80)}.dir{font-size:5vw;font-weight:900}.state{font-size:9vw;font-weight:1000}.gaps{font-size:2.6vw;font-weight:800;margin-top:12px}.reason{font-size:1.9vw;margin-top:10px}.phase{font-size:1.7vw;margin-top:8px;color:#d0e0e8}.note{margin-top:1.2vh;font-size:1.25vw;color:#9eb1ba}.sideArrow{position:absolute;top:53%;width:24vw;height:44vh;transform:translateY(-50%);opacity:.68;transition:all .12s;background:#455a64;filter:drop-shadow(0 8px 16px rgba(0,0,0,.55))}.sideArrow.left{left:2vw;clip-path:polygon(100% 0,0 50%,100% 100%,78% 50%)}.sideArrow.right{right:2vw;clip-path:polygon(0 0,100% 50%,0 100%,22% 50%)}.sideArrow.safe{background:#00c853}.sideArrow.check{background:#ffc400}.sideArrow.danger{background:#ff1744}.sideArrow.road{background:#71848f;opacity:.52}.sideArrow.turn{background:#2979ff}.sideArrow.off{background:#455a64}.sideArrow.active{opacity:1;transform:translateY(-50%) scale(1.08);filter:drop-shadow(0 0 28px rgba(255,255,255,.72))}.sideArrow.pulse{animation:pulse .72s ease-in-out infinite alternate}@keyframes pulse{from{opacity:.72}to{opacity:1}}.sideLabel{position:absolute;bottom:3vh;font-size:1.5vw;font-weight:900;color:#d8e4e9;opacity:.82}.sideLabel.left{left:9vw}.sideLabel.right{right:9vw}.safeText{color:#69f0ae}.checkText{color:#ffd740}.dangerText{color:#ff5252}.turnText{color:#64b5f6}.roadText{color:#9eb3bf}.standby{color:#aebec6}
.hudSignal{position:absolute;z-index:8;top:2.2vh;left:50%;transform:translateX(-50%);width:66vw;min-height:14vh;border:3px solid #4b6472;border-radius:22px;background:rgba(5,14,20,.97);display:flex;align-items:center;justify-content:center;gap:2.0vw;padding:1.2vh 1.5vw;box-shadow:0 8px 26px rgba(0,0,0,.55)}.hSigHead{height:9.2vh;min-width:30vw;border:3px solid #687c88;border-radius:20px;background:#11191e;padding:.8vh 1vw;display:flex;align-items:center;justify-content:center;gap:1vw}.hBulb,.hArrow{width:6.1vh;height:6.1vh;border-radius:50%;background:#273036;border:2px solid #56646d;opacity:.42;display:flex;align-items:center;justify-content:center}.hBulb.red.on{background:#ff2638;border-color:#ff9ba3;opacity:1;box-shadow:0 0 2.0vh #ff2638}.hBulb.yellow.on{background:#ffd740;border-color:#fff2a7;opacity:1;box-shadow:0 0 2.0vh #ffd740}.hBulb.green.on{background:#00e676;border-color:#9effc9;opacity:1;box-shadow:0 0 2.0vh #00e676}.hArrow{font-size:4.5vh;font-weight:1000;color:#62747e}.hArrow.on{color:#8affc1;background:#0b4a31;border-color:#79ffb5;opacity:1;box-shadow:0 0 2.0vh rgba(0,230,118,.8)}.hSigText{min-width:28vw}.hSigMain{font-size:2.4vw;font-weight:1000;line-height:1.05}.hSigSub{font-size:1.15vw;font-weight:900;margin-top:.7vh;color:#d4e1e7}.hSigTiny{font-size:.82vw;color:#8198a4;margin-top:.45vh}.hGreen{color:#57f09a}.hRed{color:#ff6673}.hYellow{color:#ffe36b}.hGray{color:#a9bac3}
</style></head><body><div class="wrap"><div class="card"><div class="hudSignal"><div class="hSigHead"><div id="hRed" class="hBulb red"></div><div id="hYellow" class="hBulb yellow"></div><div id="hArrow" class="hArrow">·</div><div id="hGreen" class="hBulb green"></div></div><div class="hSigText"><div id="hMain" class="hSigMain hGray">SIGNAL ? · MODEL STATE</div><div id="hSub" class="hSigSub">E2E probe waiting</div><div id="hTiny" class="hSigTiny">KR horizontal · * heuristic evidence</div></div></div><div id="leftA" class="sideArrow left off"></div><div id="rightA" class="sideArrow right off"></div><div class="sideLabel left">LEFT</div><div class="sideLabel right">RIGHT</div><div class="center"><div id="dir" class="dir standby">PREVIEW</div><div id="st" class="state standby">READY</div><div id="gap" class="gaps">LEFT -- · RIGHT --</div><div id="why" class="reason">삼각형 색으로 양쪽 상태 미리보기</div><div id="ph" class="phase">V53R1 FG15 + V-ASM CHECK + ROAD ? + CUT SHADOW</div></div></div><div class="note">신호표시는 E2E 진단용: RED?는 보수적 stop 후보, GREEN/GO는 stop→go edge이며 직접 램프 색 분류가 아닙니다. 화살표는 계획 회전방향입니다. 좌/우 삼각형도 SHADOW 비교용이며 제어 신호가 아닙니다.</div></div>
<script>
const leftA=document.getElementById('leftA'),rightA=document.getElementById('rightA'),dir=document.getElementById('dir'),st=document.getElementById('st'),gap=document.getElementById('gap'),why=document.getElementById('why'),ph=document.getElementById('ph'),hRed=document.getElementById('hRed'),hYellow=document.getElementById('hYellow'),hGreen=document.getElementById('hGreen'),hArrow=document.getElementById('hArrow'),hMain=document.getElementById('hMain'),hSub=document.getElementById('hSub'),hTiny=document.getElementById('hTiny');
function cls(l){l=String(l||'');if(l.startsWith('SAFE'))return'safe';if(l.startsWith('DANGER'))return'danger';if(l.startsWith('ROAD ?')||l.startsWith('CHECK ROAD')||l.startsWith('NO LANE'))return'road';if(l.startsWith('CHECK'))return'check';if(l.startsWith('TURN'))return'turn';return'off'}function textCls(l){const c=cls(l);return c==='safe'?'safeText':c==='danger'?'dangerText':c==='check'?'checkText':c==='turn'?'turnText':c==='road'?'roadText':'standby'}function reason(x){const m={'lane_change_commit_hold':'COMMIT HOLD','lane_change_rebase':'REBASE','intersection_turn_context':'TURN','commit_hold_hard_ttc_override':'TTC OVERRIDE','commit_hold_stable_hard_ttc_override':'STABLE TTC','lane_change_rebase_identity_guard':'REBASE ID GUARD','lane_change_commit_wait_for_lane':'WAIT LANE','turn_approach_no_target_lane':'TURN APPROACH','turn_wait_unconfirmed_lane':'TURN WAIT','intersection_turn_release_hold':'TURN HOLD','target_lane_absent':'NO TARGET LANE','target_lane_unconfirmed':'ROAD/LANE UNCERTAIN','front_gap<=5m':'FRONT GAP','rear_gap<=5m':'REAR GAP','boundary<=2m':'BOUNDARY','TTC<=3s':'TTC','stable_incoming_near':'INCOMING','front_gap<=12m':'FRONT GAP','rear_gap<=12m':'REAR GAP','boundary<=5m':'BOUNDARY','TTC<=5s':'TTC','outer_edge_close_watch':'EDGE WATCH','confirmed_outer_edge_forecast':'EDGE FORECAST','legacy_block_downgraded_by_fg10':'FG10 DOWNGRADE','legacy_block_downgraded_by_tts3':'TTS3 DOWNGRADE','side_conflict_now':'2D CONFLICT NOW','conflict_time<=3s':'2D≤3s','conflict_pending':'2D PENDING','conflict_3_5s_watch':'2D 3-5s','longitudinal_only_watch':'LONG ONLY','lateral_only_watch':'LAT ONLY','road_uncertain_2d_urgent':'2D URGENT','time_to_side<=3s':'TTS≤3s','side_overlap_now':'SIDE NOW','time_to_side_pending':'TTS PENDING','time_to_side_3_5s_watch':'TTS 3-5s','boundary_time_to_side_watch':'EDGE TTS','near_target_lane_watch':'NEAR WATCH','model_forecast_watch':'MODEL WATCH','road_uncertain_central_emergency':'CENTRAL EMERGENCY','road_uncertain_tts_urgent':'TTS URGENT'};return m[x]||String(x||'').replaceAll('_',' ').toUpperCase()}
function signalHud(t){t=t||{};const state=t.state||'UNKNOWN',go=Math.round(100*Number(t.go_score||0)),stop=Math.round(100*Number(t.stop_score||0));hRed.classList.remove('on');hYellow.classList.remove('on');hGreen.classList.remove('on');hArrow.classList.remove('on');hMain.className='hSigMain hGray';if(state==='GREEN_GO'){hGreen.classList.add('on');hMain.className='hSigMain hGreen';hMain.textContent=`GREEN / GO · EVID ${go}`}else if(state==='RED_CANDIDATE'){hRed.classList.add('on');hMain.className='hSigMain hRed';hMain.textContent=`RED ? · E2E STOP · ${stop}`}else if(state==='STOP_HOLD'){hMain.className='hSigMain hGray';hMain.textContent=`STOP/HOLD · LIGHT UNKNOWN · ${stop}`}else if(state==='WAIT_LEAD'){hMain.className='hSigMain hGray';hMain.textContent='LEAD AHEAD · SIGNAL UNKNOWN'}else if(state==='WATCH'){hMain.className='hSigMain hGray';hMain.textContent='STOPPED · SIGNAL UNKNOWN'}else if(state==='DRIVING'){hMain.textContent='DRIVING · SIGNAL MONITOR'}else{hMain.textContent='SIGNAL ? · MODEL STALE'}const td=t.turn_direction||'none';if(td==='left'){hArrow.textContent='←';if(state==='GREEN_GO')hArrow.classList.add('on')}else if(td==='right'){hArrow.textContent='→';if(state==='GREEN_GO')hArrow.classList.add('on')}else hArrow.textContent='·';const h=t.path_horizon_m==null?'--':Number(t.path_horizon_m).toFixed(1),j=t.horizon_jump_m==null?'--':Number(t.horizon_jump_m).toFixed(1),hold=t.stop_hold_s==null?'--':Number(t.stop_hold_s).toFixed(1);hSub.textContent=`PATH ${h}m · JUMP +${j}m · ARM ${t.stop_armed?1:0} · HOLD ${hold}s · STOP ${t.should_stop?1:0}`;hTiny.textContent=`${t.green_trigger_source&&t.green_trigger_source!=='none'?'GO '+t.green_trigger_source+' · ':''}RED? candidate only · arrow=turn path · no direct lamp classifier`}
async function tick(){try{const s=await(await fetch('/state',{cache:'no-store'})).json(),fg=s.future_gap||{},d=fg.driver_intent||{},ll=s.vasm_warning?.left?.warning_label||fg.left?.decision?.label||'--',rr=s.vasm_warning?.right?.warning_label||fg.right?.decision?.label||'--';signalHud(s.traffic_signal_probe||{});let lc=cls(ll),rc=cls(rr);if(d.active){if(d.maneuver_context==='TURN'){if(d.side==='left')lc='turn';if(d.side==='right')rc='turn'}else{const ac=cls((d.committed?d.label:s.vasm_warning?.[d.side]?.warning_label)||d.label||'CHECK ?');if(d.side==='left')lc=ac;if(d.side==='right')rc=ac}}leftA.className=`sideArrow left ${lc}${d.active&&d.side==='left'?' active':''}${d.active&&d.side==='left'&&d.phase==='COMMIT_HOLD'?' pulse':''}`;rightA.className=`sideArrow right ${rc}${d.active&&d.side==='right'?' active':''}${d.active&&d.side==='right'&&d.phase==='COMMIT_HOLD'?' pulse':''}`;if(!d.active){dir.textContent='PREVIEW';dir.className='dir standby';st.textContent='READY';st.className='state standby';gap.textContent=`LEFT ${ll} · RIGHT ${rr}`;why.textContent='양쪽 삼각형 색을 먼저 확인';ph.textContent='V53R1 FG15 + V-ASM CHECK · BLINKER OFF';}else if(d.maneuver_context==='TURN'||d.state==='TURN'){dir.textContent=d.side==='left'?'← LEFT':'RIGHT →';dir.className='dir turnText';st.textContent=(d.phase==='TURN_WAIT'||d.phase==='TURN_APPROACH')?'TURN ?':'TURN';st.className='state turnText';gap.textContent=`${(Number(fg.ego?.v_ego_mps||0)*3.6).toFixed(1)} km/h · steering ${Number(d.steering_angle_deg||0).toFixed(0)}°`;why.textContent='교차로/저속 회전으로 판단 · lane-change 판단 숨김';ph.textContent=d.phase||'TURN CONTEXT';}else{const l=(d.committed?d.label:s.vasm_warning?.[d.side]?.warning_label)||d.label||'CHECK ?';dir.textContent=d.side==='left'?'← LEFT':'RIGHT →';dir.className='dir '+textCls(l);st.textContent=l;st.className='state '+textCls(l);const f=d.front_clearance_m==null?'--':Number(d.front_clearance_m).toFixed(1),r=d.rear_clearance_m==null?'--':Number(d.rear_clearance_m).toFixed(1);const tts=d.time_to_side_s==null?'--':Number(d.time_to_side_s).toFixed(1);gap.textContent=`NOW F ${f} m · R ${r} m · 2D ${tts}s`;why.textContent=(s.vasm_warning?.[d.side]?.camera_caution?'CAM CAUTION · ':'')+((d.reasons||[]).slice(0,3).map(reason).join(' · ')||'GAP DIAGNOSTIC ONLY');ph.textContent=(d.phase||'PRECHECK')+(d.hold_remaining_s==null?'':` · HOLD ${Number(d.hold_remaining_s).toFixed(1)}s`);}}catch(e){leftA.className='sideArrow left off';rightA.className='sideArrow right off';st.textContent='CHECK DATA';st.className='state checkText';why.textContent='STATE CONNECTION LOST';}setTimeout(tick,100)}tick();
</script></body></html>'''

class H(BaseHTTPRequestHandler):
  def log_message(self,*a): pass
  def do_GET(self):
    path=self.path.split('?')[0]
    if path=='/health':
      b=json.dumps({'ok':True,'build':BUILD_VERSION,'tag':BUILD_TAG,'port':HTTP_PORT,'pid':os.getpid()},separators=(',',':')).encode()
      self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    elif path=='/state':
      mark_browser_active()
      with state_lock: b=latest_state_json
      self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    elif path=='/hud':
      mark_browser_active();b=HUD_HTML.encode()
      self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    else:
      b=HTML.encode();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)

class G80HTTPServer(ThreadingHTTPServer):
  allow_reuse_address=True
  daemon_threads=True


def _web_status(state:str, error:str=''):
  try:
    WEB_STATUS_PATH.parent.mkdir(parents=True,exist_ok=True)
    tmp=WEB_STATUS_PATH.with_suffix('.tmp')
    tmp.write_text(json.dumps({
      'state':state,'host':HTTP_HOST,'port':HTTP_PORT,'pid':os.getpid(),
      'build':BUILD_VERSION,'tag':BUILD_TAG,'error':error,'mono_ns':time.monotonic_ns()
    },separators=(',',':')))
    tmp.replace(WEB_STATUS_PATH)
  except Exception:
    pass


def http_thread():
  # V39R2: do not let a one-shot bind error silently kill the web UI forever.
  # If 28992 is temporarily occupied, record the error and retry every 2 s.
  while True:
    srv=None
    try:
      _web_status('STARTING')
      srv=G80HTTPServer((HTTP_HOST,HTTP_PORT),H)
      _web_status('LISTENING')
      srv.serve_forever(poll_interval=.5)
    except Exception as e:
      _web_status('BIND_ERROR',repr(e))
      time.sleep(2.0)
    finally:
      if srv is not None:
        try: srv.server_close()
        except Exception: pass

def atomic_write(path,text): tmp=path.with_suffix('.tmp');tmp.write_text(text);tmp.replace(path)


def _trace_aliases(o):
  vals=[]
  for k in ('key','vehicle_key','canonical_key','vehicle_anchor_key','front_key','front_link'):
    v=o.get(k)
    if v is not None and str(v): vals.append(str(v))
  for k in ('member_keys','vehicle_cluster_keys'):
    v=o.get(k)
    if isinstance(v,(list,tuple,set)):
      vals.extend(str(x) for x in v if x is not None and str(x))
  if o.get('corner_link_id') is not None:
    vals.append('corner_link:'+str(o.get('corner_link_id')))
  if o.get('corner_fused_id') is not None:
    vals.append('corner_link:'+str(o.get('corner_fused_id')))
  return set(vals)


def _display_source_domains(o):
  domains=set(str(x) for x in (o.get('canonical_domains') or []) if x)
  if domains:
    return domains
  src=str(o.get('source') or '')
  sources={src}|{str(x) for x in (o.get('vehicle_cluster_sources') or []) if x}
  if any(('front' in x) or ('fr_cmr' in x) for x in sources) or o.get('front_link') or o.get('scc_teacher_confirmed'):
    domains.add('FRONT')
  if any('corner' in x for x in sources) or o.get('corner_link_id') is not None or o.get('corner_fused_id') is not None:
    sec=str(o.get('sector') or '')
    if sec in ('FL','FR','RL','RR'):
      domains.add(sec)
    else:
      try:
        x=float(o.get('x',0)); y=float(o.get('y',0))
        if x>=0 and y>1.0: domains.add('FL')
        elif x>=0 and y<-1.0: domains.add('FR')
        elif x<0 and y>1.0: domains.add('RL')
        elif x<0 and y<-1.0: domains.add('RR')
        else: domains.add('CORNER')
      except Exception:
        domains.add('CORNER')
  if src=='c4_camera' or o.get('camera_confirmed') or o.get('camera_only'):
    domains.add('CAMERA')
  if o.get('teacher_match') or o.get('rear_teacher_confirmed'):
    domains.add('REAR_TEACHER')
  if not domains and src:
    domains.add(src.upper())
  return domains


def _decorate_display_state(objects,now_ns):
  out=[]
  for o in objects:
    d=dict(o)
    domains=_display_source_domains(d)
    d['source_mask']=sorted(domains)
    recv=int(d.get('recv_ns',0) or 0)
    age_ms=max(0.0,(int(now_ns)-recv)/1e6) if recv>0 else None
    d['source_age_ms']=None if age_ms is None else round(age_ms,1)
    physical_groups=set()
    if 'FRONT' in domains: physical_groups.add('FRONT')
    if any(x in domains for x in ('FL','FR','RL','RR','CORNER')): physical_groups.add('CORNER')
    if 'CAMERA' in domains: physical_groups.add('CAMERA')
    if 'REAR_TEACHER' in domains: physical_groups.add('REAR_TEACHER')
    if age_ms is not None and age_ms>450.0:
      state='STALE'
    elif age_ms is not None and age_ms>180.0 and bool(d.get('kalman_valid')):
      state='PREDICTED'
    elif len(physical_groups)>=2 or bool(d.get('scc_teacher_confirmed')) or bool(d.get('teacher_match')):
      state='CONFIRMED'
    else:
      state='MEASURED'
    d['display_state']=state
    out.append(d)
  return out


def _trace_diag_to_canonical(objects,canonical_objects,now_ns,view_name):
  canon=[dict(c) for c in canonical_objects]
  canon_alias=[_trace_aliases(c) for c in canon]
  out=[]
  for o in objects:
    d=dict(o); oa=_trace_aliases(d)
    best=None
    for i,c in enumerate(canon):
      ov=len(oa & canon_alias[i])
      if ov:
        score=(1000+50*ov, i, 'ALIAS')
        if best is None or score[0]>best[0]: best=score
    if best is None:
      try:
        ox=float(d.get('x')); oy=float(d.get('y')); ovx=d.get('vx'); ovx=None if ovx is None else float(ovx)
      except Exception:
        ox=oy=None; ovx=None
      if ox is not None and oy is not None:
        for i,c in enumerate(canon):
          try:
            cx=float(c.get('x')); cy=float(c.get('y')); cv=c.get('vx'); cv=None if cv is None else float(cv)
          except Exception:
            continue
          dx=abs(ox-cx); dy=abs(oy-cy); dv=0.0 if ovx is None or cv is None else abs(ovx-cv)
          gate_x=4.0 if view_name in ('front','corner') else 3.0
          gate_y=1.8 if view_name in ('front','corner') else 1.5
          if dx<=gate_x and dy<=gate_y and dv<=3.0:
            cost=(dx/gate_x)**2+(dy/gate_y)**2+.25*(dv/3.0)**2
            score=(100-cost, i, 'GEO')
            if best is None or score[0]>best[0]: best=score
    d['trace_local_key']=str(d.get('vehicle_key') or d.get('key') or '')
    if best is not None:
      c=canon[best[1]]
      d['canonical_key']=c.get('canonical_key') or c.get('vehicle_key')
      d['canonical_id']=c.get('canonical_id')
      d['canonical_valid']=True
      d['trace_match_method']=best[2]
      d['trace_canonical_domains']=list(c.get('source_mask') or c.get('canonical_domains') or [])
      # Keep local-source semantics, but use canonical state/age if it is more informative.
      d['display_state']=c.get('display_state',d.get('display_state'))
      d['source_age_ms']=c.get('source_age_ms',d.get('source_age_ms'))
    else:
      d['trace_unmatched']=True
      d['trace_match_method']='NONE'
    out.append(d)
  return _decorate_display_state(out,now_ns)


def _view_consistency(canonical,front_view,corner_view,corner_candidates):
  fk={o.get('canonical_key') for o in front_view if o.get('canonical_key')}
  ck={o.get('canonical_key') for o in corner_view if o.get('canonical_key')}
  cck={o.get('canonical_key') for o in corner_candidates if o.get('canonical_key')}
  front_need=corner_need=0
  front_missing=corner_fused_missing=corner_missing=0
  camera_only=0
  for o in canonical:
    k=o.get('canonical_key'); ds=set(o.get('source_mask') or o.get('canonical_domains') or [])
    if ds=={'CAMERA'} or (o.get('camera_only') and not any(x in ds for x in ('FRONT','FL','FR','RL','RR','CORNER'))):
      camera_only+=1
    if 'FRONT' in ds:
      front_need+=1
      if k not in fk: front_missing+=1
    if any(x in ds for x in ('FL','FR','RL','RR','CORNER')):
      corner_need+=1
      if k not in ck: corner_fused_missing+=1
      if k not in ck and k not in cck: corner_missing+=1
  return {
    'canonical_count':len(canonical),
    'canonical_front_domain':front_need,
    'canonical_corner_domain':corner_need,
    'canonical_camera_only':camera_only,
    'front_view_traced':sum(1 for o in front_view if o.get('canonical_key')),
    'corner_view_traced':sum(1 for o in corner_view if o.get('canonical_key')),
    'corner_candidates_traced':sum(1 for o in corner_candidates if o.get('canonical_key')),
    'canonical_front_without_front_view':front_missing,
    'canonical_corner_without_fused_view':corner_fused_missing,
    'canonical_corner_without_corner_view':corner_missing,
  }

_UI_OBJ_KEYS = (
  'key','source','sensor','x','y','vx','sector','front_sector','vehicle_key','canonical_key','canonical_valid',
  'canonical_primary_domain','canonical_domains','canonical_domain_history','canonical_handoff_count','canonical_reacquired','canonical_gap_ms','source_mask','source_age_ms','display_state','trace_local_key','trace_match_method','trace_unmatched','trace_canonical_domains','corner_debug_role','road_d','road_lane','road_lane_source',
  'corner_link_id','vehicle_cluster_sources','kalman_valid','kf_dormant_preserved','kf_x','kf_y','kf_vy','kf_d_dot','kf_cutin_candidate',
  'kf_cutin_confirmed','kf_ttlc_s','kf_cutin_persistence_s','kf_low_speed_lateral_candidate','kalman_trajectory',
  'imm_valid','imm_dominant_model','imm_prob_maneuver','imm_eval_age_ms','scc_teacher_confirmed','teacher_match',
  'front_link','camera_confirmed','vehicle_duplicates_merged','vehicle_member_count','shadow_role','cutin_confirmed',
  'preview_quality','raw_address','raw_slot','stage_origin','stage_gate','shadow_validation_state','shadow_reason'
)
def _ui_obj(o):
  return {k:o.get(k) for k in _UI_OBJ_KEYS if k in o and o.get(k) is not None}

_UI_TOP_KEYS = (
  'ml_case_collector','version','mono_ns','runtime_versions','runtime_mismatch','coordinate_frame','road_model','rear_teacher_match_stats',
  'canonical_tracker_stats','view_consistency_stats','web_stage_stats','web_stage_errors','kalman_motion_stats','imm_motion_stats','future_gap','traffic_signal_probe','ego_state','side_vision','front_corner_vision','lane_change_shadow','vasm_warning','cut_event_shadow','shadow_lead_interface','camera_fusion_stats',
  'shadow_leads','cut_candidates','corner_front_associations','zones','teacher_rear','scc_teacher','scc_front_match','corner_fusion_stats',
  'standard_front_preview_stats','shadow_logger','performance_stats','camera_fusion_matches'
)
def _ui_compact_core(core):
  out={k:core.get(k) for k in _UI_TOP_KEYS if k in core}
  out['sensor_fused_objects']=[_ui_obj(o) for o in core.get('sensor_fused_objects',[])]
  out['corner_fused_objects']=[_ui_obj(o) for o in core.get('corner_fused_objects',[])]
  out['corner_candidate_objects']=[_ui_obj(o) for o in core.get('corner_candidate_objects',[])[:64]]
  out['corner_debug_objects']=[_ui_obj(o) for o in core.get('corner_debug_objects',[])[:96]]
  out['front_sensor_objects']=[_ui_obj(o) for o in core.get('front_sensor_objects',[])]
  out['front_objects']=[_ui_obj(o) for o in core.get('front_objects',[])]
  out['standard_front_preview']=[_ui_obj(o) for o in core.get('standard_front_preview',[])[:48]]
  out['filtered_objects']=[_ui_obj(o) for o in core.get('filtered_objects',[])[:64]]
  out['raw_objects']=[_ui_obj(o) for o in core.get('raw_objects',[])[:96]]
  out['raw_filtered_objects']=[_ui_obj(o) for o in core.get('raw_filtered_objects',[])[:96]]
  out['selected_shadow_leads']=[_ui_obj(o) for o in core.get('selected_shadow_leads',[])[:2]]
  out['cut_candidates']=core.get('cut_candidates',[])[:5]
  out['camera_leads']=[_ui_obj(o) for o in core.get('camera_leads',[])[:12]]
  return out

def main():
  global latest_state_json
  threading.Thread(target=http_thread,daemon=True).start()
  can_sock=messaging.sub_sock('can',timeout=0,conflate=False)
  model_sock=messaging.sub_sock('modelV2',timeout=0,conflate=True)
  carstate_sock=messaging.sub_sock('carState',timeout=0,conflate=True)
  radarstate_sock=messaging.sub_sock('radarState',timeout=0,conflate=True)
  modelsp_sock=_optional_sub_sock('modelDataV2SP')
  longplansp_sock=_optional_sub_sock('longitudinalPlanSP')
  ml_case_collector = None
  ml_collector_error = ''
  try:
    ml_case_collector = MLCaseCollector()
    atexit.register(ml_case_collector.close)
  except Exception as e:
    ml_collector_error = repr(e)
  tracks=TrackStore(ttl_s=.75,continuity_s=.30);corner_fuser=CornerFusionTracker();corner_vehicle_tracker=VehicleFootprintTracker('VC');scc_matcher=SccFrontTeacherMatcher();camera_fuser=CameraRadarFusion();canonical_tracker=Canonical360Tracker('V',ttl_s=1.5);shadow_verifier=ShadowLeadVerifier();shadow_logger=_DisabledShadowLogger();front_preview=StandardFrontPreview();motion_tracker=KalmanMotionTracker('KF');imm_tracker=ImmMotionTracker();future_gap=FutureGapEvaluator();vasm_warning_evaluator=VASMWarningEvaluator();signal_probe=TrafficSignalProbe();camera_leads=[];model_path=[];road_model={};model_path_recv_ns=0;v_ego=0.0;a_ego=0.0;steering_angle_deg=0.0;left_blinker=False;right_blinker=False;v_ego_recv_ns=0;production=None;rear_teacher=[];rear_teacher_by_bus={};scc_teacher=None;scc_teacher_by_bus={};SCC_BUS=int(os.getenv('G80_SCC_BUS',str(DEFAULT_SCC_BUS)))
  udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
  vision_rx=None;vision_rx_error=''
  try:
    vision_rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);vision_rx.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);vision_rx.bind(('127.0.0.1',SIDE_VISION_UDP_PORT));vision_rx.setblocking(False)
  except OSError as e:
    vision_rx_error=repr(e);vision_rx=None
  side_vision_state=_blank_side_vision(vision_rx_error or 'waiting_for_g80sidevision')
  front_corner_rx=None;front_corner_rx_error=''
  try:
    front_corner_rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);front_corner_rx.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);front_corner_rx.bind(('127.0.0.1',FRONT_CORNER_VISION_UDP_PORT));front_corner_rx.setblocking(False)
  except OSError as e:
    front_corner_rx_error=repr(e);front_corner_rx=None
  front_corner_vision_state=_blank_front_corner_vision(front_corner_rx_error or 'waiting_for_g80frontcornervision')
  bsd_monitor=BsdMonitor()
  radar_recv={'corner_A':0,'corner_B':0,'front':0}
  carstate_valid=False
  diag={'corner_A_frames':0,'corner_B_frames':0,'corner_decoded_total':0,'corner_rear_total':0,'corner_front_total':0,'corner_A_by_bus':{},'corner_B_by_bus':{},'scc_frames_by_bus':{},'scc_teacher_updates':0,'model_frames':0,'camera_leads_latest':0,'model_transport_lag_ms':None,'last_transport_lag_ms':None}
  period=1.0/max(PUBLISH_HZ,1.0);next_pub=time.monotonic();next_debug_write=0.0;next_ui_update=0.0;next_web_stage_audit=0.0;web_stage_stats={};web_stage_errors=[];last_pub_ns=0;last_processing_ms=0.0;last_udp_ms=0.0;last_ui_ms=0.0;last_ui_kb=0.0;last_stage_ms={};last_can_drain_ms=0.0;last_can_msgs=0;can_guard_breaks=0;publish_late_count=0;scheduler_skips=0;last_publish_late_ms=0.0
  while True:
    processed=0;guard_hit=False;can_drain_t0=time.perf_counter()
    can_deadline=next_pub-CAN_DRAIN_GUARD_S
    for _ in range(2000):
      # V50R1/V51: never let a large CAN backlog consume the publication slot.
      # Unread CAN remains queued for the following outer-loop iteration.
      if time.monotonic() >= can_deadline:
        guard_hit=True
        break
      msg=messaging.recv_one_or_none(can_sock)
      if msg is None: break
      recv_ns=time.monotonic_ns();can_log_ns=int(msg.logMonoTime);diag['last_transport_lag_ms']=round((recv_ns-can_log_ns)/1e6,3)
      for f in msg.can:
        bus,addr,dat=int(f.src),int(f.address),bytes(f.dat)
        if -50_000_000 <= recv_ns-can_log_ns <= 500_000_000:
          if addr in CORNER_A: radar_recv['corner_A']=recv_ns
          if addr in CORNER_B: radar_recv['corner_B']=recv_ns
          if addr in FR_CMR and bus==2: radar_recv['front']=recv_ns
        if addr==SCC_CONTROL_ADDR:
          sb=str(bus);diag['scc_frames_by_bus'][sb]=diag['scc_frames_by_bus'].get(sb,0)+1
          st=decode_scc_teacher(dat,can_log_ns,recv_ns,bus)
          if st is not None:
            scc_teacher_by_bus[bus]=st
            diag['scc_teacher_updates']+=1
        if addr in CORNER_A:
          diag['corner_A_frames']+=1;s=str(bus);diag['corner_A_by_bus'][s]=diag['corner_A_by_bus'].get(s,0)+1
        if addr in CORNER_B:
          diag['corner_B_frames']+=1;s=str(bus);diag['corner_B_by_bus'][s]=diag['corner_B_by_bus'].get(s,0)+1
        if addr in CORNER_A or addr in CORNER_B:
          o=decode_corner24(addr,dat,can_log_ns,recv_ns)
          if o:
            od=o.to_dict();tracks.update(od);diag['corner_decoded_total']+=1
            if od['x']<-.5:diag['corner_rear_total']+=1
            elif od['x']>.5:diag['corner_front_total']+=1
        elif addr in FRONT_GROUP1 and bus==0:
          for o in decode_front_group1_candidate(addr,dat,can_log_ns,recv_ns): tracks.update(o.to_dict())
        elif addr in FR_CMR and bus==2:
          for o in decode_fr_cmr_reference(addr,dat,can_log_ns,recv_ns): tracks.update(o.to_dict())
        elif addr==0x1EA:
          rt=decode_rear_teacher_1ea(dat)
          if rt:
            for e in rt:
              e['bus']=bus;e['recv_ns']=recv_ns
            rear_teacher_by_bus[bus]=rt
      processed+=1
    last_can_drain_ms=(time.perf_counter()-can_drain_t0)*1000.0;last_can_msgs=processed
    if guard_hit:
      can_guard_breaks+=1

    mmsg=messaging.recv_one_or_none(model_sock)
    if mmsg is not None:
      mrecv_ns=time.monotonic_ns()
      camera_leads=decode_model_leads(mmsg.modelV2,int(mmsg.logMonoTime),mrecv_ns)
      road_model=extract_road_model(mmsg.modelV2,mrecv_ns)
      signal_probe.update_model(mmsg.modelV2,mrecv_ns)
      model_path=path_as_tuples(road_model)
      model_path_recv_ns=mrecv_ns
      diag['model_frames']+=1
      diag['camera_leads_latest']=len(camera_leads)
      diag['model_transport_lag_ms']=round((mrecv_ns-int(mmsg.logMonoTime))/1e6,3)

    csmsg=messaging.recv_one_or_none(carstate_sock)
    if csmsg is not None:
      cs_recv_ns=time.monotonic_ns()
      # V51 collector: keep high-value ego dynamics synchronized with 10 Hz cases.
      try:
        if ml_case_collector is not None:
          ml_case_collector.update_ego(csmsg.carState,cs_recv_ns,int(csmsg.logMonoTime),bool(csmsg.valid))
      except Exception:
        pass
      bsd_monitor.refresh_support(cs_recv_ns)
      bsd_monitor.update(csmsg.carState,cs_recv_ns,int(csmsg.logMonoTime),bool(csmsg.valid))
      carstate_valid=bool(bsd_monitor.valid)
      try:
        v_ego=float(csmsg.carState.vEgo)
        a_ego=float(csmsg.carState.aEgo)
        steering_angle_deg=float(csmsg.carState.steeringAngleDeg)
        left_blinker=bool(csmsg.carState.leftBlinker)
        right_blinker=bool(csmsg.carState.rightBlinker)
        v_ego_recv_ns=time.monotonic_ns()
        signal_probe.update_carstate(csmsg.carState,v_ego_recv_ns)
      except Exception: pass

    if modelsp_sock is not None:
      tsp=messaging.recv_one_or_none(modelsp_sock)
      if tsp is not None:
        signal_probe.update_turn(tsp,time.monotonic_ns())
    if longplansp_sock is not None:
      lpsp=messaging.recv_one_or_none(longplansp_sock)
      if lpsp is not None:
        signal_probe.update_longitudinal_plan_sp(lpsp,time.monotonic_ns())

    rsmsg=messaging.recv_one_or_none(radarstate_sock)
    if rsmsg is not None:
      production=snapshot_production_radar_state(rsmsg,time.monotonic_ns())

    side_vision_state=_drain_side_vision(vision_rx,side_vision_state,time.monotonic_ns())
    front_corner_vision_state=_drain_front_corner_vision(front_corner_rx,front_corner_vision_state,time.monotonic_ns())
    now=time.monotonic()
    if now>=next_pub:
      last_publish_late_ms=max(0.0,(now-next_pub)*1000.0)
      if last_publish_late_ms>10.0:
        publish_late_count+=1
      now_ns=time.monotonic_ns()
      scc_teacher=choose_scc_teacher(scc_teacher_by_bus,SCC_BUS,now_ns)
      fresh_rear={}
      for rb,rv in list(rear_teacher_by_bus.items()):
        if rv and now_ns-int(rv[0].get('recv_ns',0))<=500_000_000:
          fresh_rear[rb]=rv
      if 1 in fresh_rear:
        rear_teacher=fresh_rear[1]
      else:
        usable=[rv for rv in fresh_rear.values() if any(x.get('teacher_usable') for x in rv)]
        rear_teacher=usable[0] if usable else (next(iter(fresh_rear.values())) if fresh_rear else [])
      pub_start=time.perf_counter(); stage_ms={}
      ui_active=browser_is_active(now_ns); debug_due=DEBUG_STATE_HZ>0.0 and now>=next_debug_write
      # V51: monitor-only stage checks run only on browser frames or a 2Hz
      # shadow-audit heartbeat; never duplicate the raw gates at 10Hz off-screen.
      web_audit_due=debug_due or (ui_active and now>=next_ui_update) or now>=next_web_stage_audit
      st=time.perf_counter()
      raw=tracks.snapshot(now_ns);filt=filtered_objects(raw,rear_teacher);web_filtered_diag=diagnostic_raw_filtered(raw,rear_teacher) if web_audit_due else [];cf=corner_fuser.update(filt,now_ns);fronts=build_front_objects(filt);combined=associate_corner_front(cf['corner_fused_objects'],fronts);corners=combined['corner_objects'];fronts=combined['front_objects'];radar_all_fused=combined['all_fused_objects'];assocs=combined['associations'];camf=camera_fuser.update(radar_all_fused,camera_leads,now_ns,fronts);sensor_fused_clusters=camf['sensor_fused_objects'];front_sensor=camf['front_sensor_objects'];corner_vehicle,corner_vehicle_stats=corner_vehicle_tracker.update(corners,now_ns);stdp=front_preview.update(raw,fronts,camera_leads,scc_teacher,now_ns)
      stage_ms['core_fusion']=(time.perf_counter()-st)*1000.0
      st=time.perf_counter();road_view=road_model_with_age(road_model,now_ns)
      # V34: road projection is authoritative only for the Canonical sensor-fused
      # set. Diagnostic RAW/FILTERED/CORNER/FRONT tabs keep ego-frame x/y without
      # repeated Frenet projection; this removes a dense-traffic UI cost.
      raw_view=annotate_raw(raw) if (ui_active or debug_due) else []
      raw_filtered_view=web_filtered_diag if (ui_active or debug_due) else []
      filt_view=filt if (ui_active or debug_due) else []
      corner_candidates_raw=[dict(o,corner_debug_role='CANDIDATE') for o in filt if o.get('source')=='corner24']
      sensor_fused_road=annotate_objects(sensor_fused_clusters,road_view)
      corner_vehicle_road=[dict(o,corner_debug_role='FUSED') for o in corner_vehicle];corners_view=corners;fronts_view=fronts
      radar_all_view=radar_all_fused;front_sensor_road=front_sensor
      std_points_view=stdp['points'];camera_leads_view=camera_leads
      stage_ms['road_annotate']=(time.perf_counter()-st)*1000.0
      st=time.perf_counter();sensor_fused_road,scc_match_status=scc_matcher.update(sensor_fused_road,scc_teacher,now_ns);canonical_road,canonical_stats=canonical_tracker.update(sensor_fused_road,now_ns);stage_ms['teacher_canonical']=(time.perf_counter()-st)*1000.0
      st=time.perf_counter();sensor_fused_view,kalman_stats=motion_tracker.update(canonical_road,road_view,now_ns,v_ego);stage_ms['kf']=(time.perf_counter()-st)*1000.0
      st=time.perf_counter();sensor_fused_view,imm_stats=imm_tracker.update(sensor_fused_view,road_view,now_ns,v_ego);sensor_fused_view=_decorate_display_state(sensor_fused_view,now_ns);stage_ms['imm']=(time.perf_counter()-st)*1000.0
      bsd_state=bsd_monitor.snapshot(now_ns)
      monitor_health={'radar_frames_fresh':all(t and 0<=now_ns-t<=700_000_000 for t in radar_recv.values()),
                      'radar_frame_age_ms':{k:None if not t else round((now_ns-t)/1e6,1) for k,t in radar_recv.items()},
                      'carstate_fresh':bool(carstate_valid and v_ego_recv_ns and 0<=now_ns-v_ego_recv_ns<=500_000_000),
                      'limitation':'frame liveness only, not proof of complete sensor coverage'}
      st=time.perf_counter();future_gap_state=future_gap.update(sensor_fused_view,v_ego,a_ego,left_blinker,right_blinker,now_ns,steering_angle_deg=steering_angle_deg,road_curve_direction=road_view.get('curve_direction'),road_model=road_view,bsd=bsd_state,monitor_health=monitor_health);stage_ms['future_gap']=(time.perf_counter()-st)*1000.0
      has_prod_lead=bool(production and (production.get('leadOne') or {}).get('status'))
      traffic_signal_state=signal_probe.snapshot(now_ns,has_lead=has_prod_lead)
      st=time.perf_counter();shadow=shadow_verifier.update(sensor_fused_view,model_path,v_ego,now_ns,production,model_path_recv_ns,v_ego_recv_ns,scc_teacher,camera_leads=camera_leads);stage_ms['shadow']=(time.perf_counter()-st)*1000.0
      selected_leads_view=selected_shadow_leads(shadow,sensor_fused_view)
      lead_shadow_bridge=build_shadow_lead_interface(shadow,selected_leads_view,now_ns)
      cut_event_shadow=make_cut_event_shadow(sensor_fused_view,shadow,now_ns)
      cut_candidates=cut_event_shadow['candidates']
      lane_change_shadow=build_lane_change_shadow(future_gap_state,side_vision_state,front_corner_vision_state,now_ns)
      vasm_warning=vasm_warning_evaluator.update(future_gap_state,side_vision_state,front_corner_vision_state,now_ns, enabled=not os.path.exists('/data/radar/DISABLE_G80_VASM_WARNING'))
      # V39: Canonical360 stays full coverage; selective KF4/IMM3 feed FG8 at the 10 Hz publication loop.
      corner_vehicle_view=_trace_diag_to_canonical(corner_vehicle_road,sensor_fused_view,now_ns,'corner');corner_kalman_stats={'disabled_in_v32':True}
      corner_candidate_view=_trace_diag_to_canonical(corner_candidates_raw,sensor_fused_view,now_ns,'corner')
      corner_debug_view=corner_vehicle_view+corner_candidate_view
      front_sensor_view=_trace_diag_to_canonical(front_sensor_road,sensor_fused_view,now_ns,'front');front_kalman_stats={'disabled_in_v32':True}
      fronts_view=_trace_diag_to_canonical(fronts_view,sensor_fused_view,now_ns,'front')
      std_points_view=_trace_diag_to_canonical(std_points_view,sensor_fused_view,now_ns,'stdpreview')
      filt_view=_trace_diag_to_canonical(filt_view,sensor_fused_view,now_ns,'filtered') if (ui_active or debug_due) else []
      view_consistency_stats=_view_consistency(sensor_fused_view,front_sensor_view,corner_vehicle_view,corner_candidate_view)
      web_audit_start=time.perf_counter()
      if web_audit_due:
        web_stage_stats=make_stage_audit(raw,web_filtered_diag,filt,corner_vehicle_view,front_sensor_view,sensor_fused_view,selected_leads_view,std_points_view)
        web_stage_errors=validate_stage_contract(raw,web_filtered_diag,corner_vehicle_view,front_sensor_view,sensor_fused_view,selected_leads_view,std_points_view)
        next_web_stage_audit=now+0.5
      stage_ms['web_stage_audit']=(time.perf_counter()-web_audit_start)*1000.0 if web_audit_due else 0.0
      # V30 downstream occupancy/teacher summaries use the authoritative canonical objects.
      zones=occupied_zones(sensor_fused_view)
      rear_matches=[]
      for o in sensor_fused_view:
        if o.get('teacher_match') or o.get('rear_teacher_confirmed'):
          rear_matches.append({'sector':o.get('rear_teacher_sector') or ('LR' if float(o.get('y',0))>0 else 'RR'),
                               'key':o.get('vehicle_key') or o.get('key'),
                               'error_m':o.get('teacher_error_m'),
                               'teacher_distance_m':o.get('rear_teacher_distance_m'),
                               'predicted_distance_m':o.get('rear_teacher_predicted_distance_m')})
      rear_teacher_match_stats={'usable_teacher_count':sum(1 for t in rear_teacher if t.get('teacher_usable')),
                                'matched_count':len(rear_matches),'matches':rear_matches,
                                'errors_m':[m.get('error_m') for m in rear_matches if m.get('error_m') is not None],
                                'one_to_one':True,'match_gate_m':0.8}
      coordinate_frame={'x_zero':COORDINATE_X_ORIGIN,'x_positive':'forward','y_positive':'left',
                        'ego_display_length_m':EGO_DISPLAY_LENGTH_M,'ego_display_center_x_m':EGO_DISPLAY_CENTER_X_M,
                        'decoded_object_x_adjustment_m':OBJECT_X_ADJUSTMENT_M,
                        'object_x_preserved':True,'note':'ego pictogram front edge is x=0; decoded object x is unchanged'}
      # Core state is always produced because radar fusion/teacher logic and the
      # Android UDP stream must remain live even with no browser connected.
      runtime_versions={'build':BUILD_VERSION,'tag':BUILD_TAG,'logger':LOGGER_SERVICE_VERSION,'kalman_api':KALMAN_API_VERSION,'imm_api':IMM_API_VERSION,'future_gap_api':FUTURE_GAP_API_VERSION,'android_protocol':PROTOCOL_VERSION,'side_vision_api':BUILD_SIDE_VISION_API_VERSION,'front_corner_vision_api':BUILD_FRONT_CORNER_VISION_API_VERSION}
      runtime_mismatch=(BUILD_VERSION!=52 or LOGGER_SERVICE_VERSION!=BUILD_VERSION or KALMAN_API_VERSION!=4 or IMM_API_VERSION!=3 or FUTURE_GAP_API_VERSION!=15 or BUILD_FUTURE_GAP_API_VERSION!=15 or PROTOCOL_VERSION!=21 or BUILD_SIDE_VISION_API_VERSION!=2 or BUILD_FRONT_CORNER_VISION_API_VERSION!=1)
      core={'version':BUILD_VERSION,'mono_ns':now_ns,'runtime_versions':runtime_versions,'runtime_mismatch':runtime_mismatch,'objects':sensor_fused_view,
            'sensor_fused_objects':sensor_fused_view,'all_fused_objects':sensor_fused_view,'canonical360_objects':sensor_fused_view,
            'radar_fused_objects':radar_all_view,
            'web_stage_stats':web_stage_stats,'web_stage_errors':web_stage_errors,
            'raw_filtered_objects':raw_filtered_view,'selected_shadow_leads':selected_leads_view,'cut_event_shadow':cut_event_shadow,'cut_candidates':cut_candidates,'shadow_lead_interface':lead_shadow_bridge,'lane_change_shadow':lane_change_shadow,'vasm_warning':vasm_warning,
            'corner_fused_objects':corner_vehicle_view,'corner_candidate_objects':corner_candidate_view,'corner_debug_objects':corner_debug_view,'corner_radar_objects':corners_view,'front_objects':fronts_view,'front_sensor_objects':front_sensor_view,
            'standard_front_preview':std_points_view,'standard_front_preview_stats':stdp['stats'],
            'camera_leads':camera_leads_view,'camera_fusion_matches':camf['camera_matches'],
            'road_model':road_view,'coordinate_frame':coordinate_frame,'rear_teacher_match_stats':rear_teacher_match_stats,
            'canonical_tracker_stats':canonical_stats,'view_consistency_stats':view_consistency_stats,'kalman_motion_stats':dict(kalman_stats,canonical_identity_input=True),'imm_motion_stats':imm_stats,'future_gap':future_gap_state,'traffic_signal_probe':traffic_signal_state,'ego_state':{'vEgo':v_ego,'aEgo':a_ego,'steeringAngleDeg':steering_angle_deg,'leftBlinker':left_blinker,'rightBlinker':right_blinker},'side_vision':side_vision_state,'front_corner_vision':front_corner_vision_state,'corner_kalman_motion_stats':corner_kalman_stats,'front_kalman_motion_stats':front_kalman_stats,
            'camera_fusion_stats':dict(camf['stats'],corner_vehicle_objects_before=corner_vehicle_stats['vehicle_objects_before'],corner_vehicle_objects_after=corner_vehicle_stats['vehicle_objects_after'],corner_vehicle_duplicates_merged=corner_vehicle_stats['vehicle_duplicates_merged']),'shadow_leads':shadow,
            'corner_front_associations':assocs,'zones':zones,
            'teacher_rear':rear_teacher,'scc_teacher':scc_teacher or {},
            'scc_teacher_by_bus':{str(k):v for k,v in scc_teacher_by_bus.items()},
            'scc_front_match':scc_match_status,'corner_fusion_stats':cf['stats'],
            'shadow_logger':shadow_logger.status()}
      # Event-labelled RAM history runs even with the browser asleep.
      # Time it explicitly: if the 10 Hz source loop cannot keep cadence we need
      # to distinguish collector cost from fusion/UI/vision-process contention.
      ml_t0=time.perf_counter()
      try:
        if ml_case_collector is not None:
          ml_case_collector.update(core, now_ns, raw, filt)
          core['ml_case_collector'] = ml_case_collector.status()
        else:
          core['ml_case_collector'] = {'enabled': False, 'last_error': ml_collector_error}
      except Exception as e:
        core['ml_case_collector'] = {'enabled': False, 'last_error': repr(e)}
      stage_ms['ml_collector']=(time.perf_counter()-ml_t0)*1000.0
      interval_ms=None if last_pub_ns==0 else (now_ns-last_pub_ns)/1e6
      core['performance_stats']={'processing_ms':round(last_processing_ms,2),'publish_interval_ms':None if interval_ms is None else round(interval_ms,2),'target_hz':PUBLISH_HZ,'can_drain_ms':round(last_can_drain_ms,2),'can_msgs_this_loop':int(last_can_msgs),'can_drain_guard_ms':round(CAN_DRAIN_GUARD_S*1000.0,2),'can_drain_guard_breaks':int(can_guard_breaks),'publish_late_ms':round(last_publish_late_ms,2),'publish_late_count':int(publish_late_count),'scheduler_skips':int(scheduler_skips),'last_can_transport_lag_ms':diag.get('last_transport_lag_ms'),'local_kf_replicas_disabled':True,'imm_target_hz':imm_stats.get('target_hz',2.5),'stage_ms':{k:round(v,2) for k,v in stage_ms.items()},'udp_json_ms':round(last_udp_ms,2),'ui_json_ms':round(last_ui_ms,2),'logger_event_policy':'V52R7: ML-only cases; lead selection SHADOW only'}

      # V51: continuous shadow/golden disk logging is hard-disabled.
      core['shadow_logger']=shadow_logger.status()

      # Android/render protocol stays active independently of the browser.
      udp_t0=time.perf_counter()
      try:
        udp.sendto(json.dumps(build_render_packet(core),separators=(',',':')).encode(),(UDP_HOST,UDP_PORT))
      except OSError:
        pass
      last_udp_ms=(time.perf_counter()-udp_t0)*1000.0

      # V35: compact browser state is cached at 8 Hz.  The core loop remains
      # independent; duplicate canonical arrays are not serialized four times.
      ui_t0=time.perf_counter()
      ui_should_update=((ui_active and now>=next_ui_update) or debug_due)
      if ui_should_update:
        out=_ui_compact_core(core)
        out.update({
          'filtered_objects':[_ui_obj(o) for o in filt_view[:64]],
          'raw_objects':[_ui_obj(o) for o in raw_view[:96]],
          'raw_filtered_objects':[_ui_obj(o) for o in raw_filtered_view[:96]],
          'selected_shadow_leads':[_ui_obj(o) for o in selected_leads_view[:2]],'cut_candidates':cut_candidates[:5],
          'web_stage_stats':web_stage_stats,'web_stage_errors':web_stage_errors,
          'diagnostics':dict(diag,ui_sleeping=not ui_active),
          'browser_active':ui_active,
          'notes':{
            'ui':'v53r1-road-separate-vasm-check-only-hud-shadow',
            'display_scale':'SHORT/LONG 1:1 + WIDE + DRIVE perspective; source-traceable Canonical C4-road overlay',
            'browser_policy':'8Hz cached seven-stage browser data; RAW FILTERED does not cross-source dedup; no control changes',
            'udp':'Android final sensor-fusion packet on 28991',
            'control':'disabled',
            'integration':'NOT connected to RadarInterface/radarTracks/radard',
            'future_front':'plain-dict RadarPoint preview only',
            'future_gap':'FG15 BSD + target-lane + synchronized conflict evaluation + raw commit latch; seven-stage audit',
            'identity':'Canonical360Tracker is the only global Vxxxx authority; diagnostic views are traced back to Vxxxx when possible',
            'motion_prediction':'Selective KF4 + dormant preserve; IMM3 cached trajectories consumed by FG15',
            'runtime_versions':runtime_versions,'runtime_mismatch':runtime_mismatch,
            'shadow_control':'NEVER publishes radarTracks/radarState / NEVER CAN TX',
            'debug_state_hz':DEBUG_STATE_HZ,'ui_state_hz':UI_STATE_HZ,'shadow_log_dir':'OFF','side_vision':'C4 native CABIN VisionIPC side-window V-ASM classifier; SHADOW only; setup port 28994','front_corner_vision':'C4 wide-road FL/FR ROI classifier; side V-ASM reused initially; SHADOW only; setup port 28995'
          }
        })
        out['performance_stats']=dict(out.get('performance_stats',{}),ui_json_kb=round(last_ui_kb,2),ui_state_hz=UI_STATE_HZ)
        if debug_due:
          try:
            atomic_write(STATE_PATH,json.dumps(out,separators=(',',':')))
          except OSError:
            pass
          next_debug_write=now+1.0/max(DEBUG_STATE_HZ,0.1)
        b=json.dumps(out,separators=(',',':')).encode()
        last_ui_kb=len(b)/1024.0
        with state_lock:
          latest_state.clear();latest_state.update(out);latest_state_json=b
        next_ui_update=now+1.0/max(UI_STATE_HZ,1.0)
      elif not ui_active and now>=next_ui_update:
        compact={'version':BUILD_VERSION,'mono_ns':now_ns,'runtime_versions':runtime_versions,'runtime_mismatch':runtime_mismatch,'browser_active':False,
                 'performance_stats':dict(core['performance_stats'],ui_json_kb=round(last_ui_kb,2),ui_state_hz=UI_STATE_HZ),
                 'ml_case_collector':core.get('ml_case_collector',{}),'side_vision':core.get('side_vision',{}),'front_corner_vision':core.get('front_corner_vision',{}),'lane_change_shadow':core.get('lane_change_shadow',{}),'vasm_warning':core.get('vasm_warning',{}),'shadow_lead_interface':core.get('shadow_lead_interface',{}),'cut_event_shadow':core.get('cut_event_shadow',{}),'cut_candidates':core.get('cut_candidates',[]),'shadow_logger':core.get('shadow_logger',{}),'diagnostics':dict(diag,ui_sleeping=True)}
        b=json.dumps(compact,separators=(',',':')).encode()
        last_ui_kb=len(b)/1024.0
        with state_lock:
          latest_state.clear();latest_state.update(compact);latest_state_json=b
        next_ui_update=now+0.5
      last_ui_ms=(time.perf_counter()-ui_t0)*1000.0 if ui_should_update else 0.0
      last_stage_ms=stage_ms
      last_processing_ms=(time.perf_counter()-pub_start)*1000.0
      interval_ms=None if last_pub_ns==0 else (now_ns-last_pub_ns)/1e6
      last_pub_ns=now_ns
      # Fixed-phase scheduler: preserve the 100 ms phase and count skipped slots.
      next_pub+=period;end_now=time.monotonic()
      if next_pub<end_now:
        skipped=(int((end_now-next_pub)/period)+1)
        scheduler_skips+=skipped
        next_pub += skipped*period
    if processed==0: time.sleep(.002)
if __name__=='__main__': main()
