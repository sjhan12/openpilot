#!/usr/bin/env python3
from __future__ import annotations
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
from openpilot.selfdrive.g80_radar.shadow_logger import ShadowLogger, LOGGER_SERVICE_VERSION
from openpilot.selfdrive.g80_radar.front_standard_preview import StandardFrontPreview
from openpilot.selfdrive.g80_radar.road_geometry import extract_road_model,road_model_with_age,path_as_tuples,annotate_objects
from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION, BUILD_TAG, COORDINATE_X_ORIGIN, EGO_DISPLAY_LENGTH_M, EGO_DISPLAY_CENTER_X_M, OBJECT_X_ADJUSTMENT_M, FUTURE_GAP_API_VERSION as BUILD_FUTURE_GAP_API_VERSION
from openpilot.selfdrive.g80_radar.kalman_motion import KalmanMotionTracker, KALMAN_API_VERSION
from openpilot.selfdrive.g80_radar.canonical_tracker import Canonical360Tracker
from openpilot.selfdrive.g80_radar.imm_motion import ImmMotionTracker, IMM_API_VERSION
from openpilot.selfdrive.g80_radar.future_gap import FutureGapEvaluator, FUTURE_GAP_API_VERSION
from openpilot.selfdrive.g80_radar.traffic_signal_probe import TrafficSignalProbe

UDP_HOST=os.getenv('G80_RADAR_UDP_HOST','255.255.255.255')
UDP_PORT=int(os.getenv('G80_RADAR_UDP_PORT','28991'))
HTTP_PORT=int(os.getenv('G80_RADAR_HTTP_PORT','28992'))
STATE_PATH=Path(os.getenv('G80_RADAR_STATE_PATH','/dev/shm/g80_radar.json'))
PUBLISH_HZ=float(os.getenv('G80_RADAR_PUBLISH_HZ','10'))
state_lock=threading.Lock()
browser_lock=threading.Lock()
browser_last_poll_ns=0
BROWSER_ACTIVE_NS=int(float(os.getenv('G80_BROWSER_ACTIVE_SEC','2.0'))*1e9)
DEBUG_STATE_HZ=max(0.0,float(os.getenv('G80_DEBUG_STATE_HZ','0')))
UI_STATE_HZ=max(2.0,float(os.getenv('G80_UI_STATE_HZ','8.0')))

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

HTML=r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover"><title>G80 5-Radar V38R2 SOURCE TRACE + ROAD GATE + SIGNAL PROBE</title><style>
:root{--bg:#061018;--panel:#091720;--line:#24404f;--text:#eaf4f8;--muted:#8fa8b5;--cyan:#16d4e3;--green:#57d96a;--gold:#ffd34e;--purple:#cf4eff;--orange:#ff913c;--blue:#2979ff}*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%;overflow:hidden;background:radial-gradient(circle at 50% 0,#0d2432 0,#061018 46%,#03090e 100%);color:var(--text);font-family:Arial,"Noto Sans KR",sans-serif}body{display:flex;flex-direction:column}.top{height:58px;display:flex;align-items:center;gap:7px;padding:7px 10px;background:rgba(4,13,19,.96);border-bottom:1px solid #2d4b5c}.tabs,.range{display:flex;gap:5px}.tabs button,.range button,#fs{border:1px solid #35566a;border-radius:8px;background:#0a1822;color:#b9cad4;padding:9px 13px;font-size:12px;font-weight:700}.tabs button.active,.range button.active{color:white;background:linear-gradient(#118eff,#0865c2);border-color:#63bcff;box-shadow:inset 0 0 0 1px #6bc2ff}.spacer{flex:1}.brand{text-align:right;line-height:1.12;margin-right:8px}.brand b{font-size:16px}.brand em{font-style:normal;color:var(--cyan);font-weight:700}.live{font-size:11px;color:#68ff83;border:1px solid #28513a;padding:5px 8px;border-radius:8px}.main{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) 342px;gap:8px;padding:8px}.scene,.side{border:1px solid #29495a;border-radius:14px;background:linear-gradient(180deg,rgba(8,23,32,.96),rgba(4,13,19,.96));overflow:hidden}.scene{position:relative}.scene canvas{width:100%;height:100%;display:block}.intentBanner{position:absolute;z-index:20;left:50%;top:12px;transform:translateX(-50%);min-width:360px;max-width:72%;padding:10px 18px;border-radius:14px;border:2px solid #52636d;background:rgba(8,18,25,.94);box-shadow:0 6px 24px rgba(0,0,0,.4);text-align:center;pointer-events:none;opacity:.28}.intentBanner.active{opacity:.98}.intentBanner.safe{background:rgba(0,83,40,.94);border-color:#00e676}.intentBanner.check{background:rgba(102,76,0,.95);border-color:#ffd54f}.intentBanner.danger{background:rgba(112,14,14,.96);border-color:#ff4b4b}.intentBanner.turn{background:rgba(12,62,112,.96);border-color:#4aa3ff}.intentBanner .intentMain{font-size:30px;font-weight:900}.intentBanner .intentSub{margin-top:4px;font-size:12px}.intentBanner .intentTiny{margin-top:3px;font-size:10px;color:#d4e2e8}.intentBanner .arrow{font-size:34px}
.sideArrow{position:absolute;z-index:19;top:50%;width:92px;height:132px;transform:translateY(-50%);opacity:.62;pointer-events:none;transition:opacity .12s,transform .12s,filter .12s;background:#455a64;filter:drop-shadow(0 4px 10px rgba(0,0,0,.55))}
.sideArrow.left{left:14px;clip-path:polygon(100% 0,0 50%,100% 100%,78% 50%)}
.sideArrow.right{right:14px;clip-path:polygon(0 0,100% 50%,0 100%,22% 50%)}
.sideArrow.safe{background:#00c853}.sideArrow.check{background:#ffc400}.sideArrow.danger{background:#ff1744}.sideArrow.turn{background:#2979ff}.sideArrow.off{background:#455a64}
.sideArrow.active{opacity:.98;filter:drop-shadow(0 0 18px rgba(255,255,255,.72));transform:translateY(-50%) scale(1.16)}
.sideArrow.pulse{animation:arrowPulse .75s ease-in-out infinite alternate}
@keyframes arrowPulse{from{opacity:.72}to{opacity:1}}

.signalProbe{position:absolute;z-index:22;right:16px;top:12px;width:268px;min-height:94px;display:flex;align-items:center;gap:10px;padding:8px 10px;border:2px solid #415764;border-radius:14px;background:rgba(7,15,20,.94);box-shadow:0 6px 22px rgba(0,0,0,.42);pointer-events:none}.tlHead{width:42px;border:2px solid #586873;border-radius:12px;background:#11191e;padding:5px 6px;display:flex;flex-direction:column;gap:4px;align-items:center}.tlBulb{width:22px;height:22px;border-radius:50%;background:#273036;border:1px solid #56646d;opacity:.42}.tlBulb.red.on{background:#ff2638;border-color:#ff808b;opacity:1;box-shadow:0 0 13px #ff2638}.tlBulb.yellow.on{background:#ffd740;border-color:#fff0a0;opacity:1;box-shadow:0 0 13px #ffd740}.tlBulb.green.on{background:#00e676;border-color:#8fffc0;opacity:1;box-shadow:0 0 13px #00e676}.tlArrow{width:42px;height:28px;border:1px solid #4a5d68;border-radius:8px;display:flex;align-items:center;justify-content:center;font-size:23px;font-weight:1000;color:#657681;background:#101a20}.tlArrow.on{color:#7ec8ff;border-color:#4aa3ff;background:#0e3557;box-shadow:0 0 10px rgba(74,163,255,.7)}.sigText{flex:1;min-width:0}.sigMain{font-size:18px;font-weight:1000;line-height:1.1;white-space:nowrap}.sigSub{font-size:10.5px;font-weight:800;margin-top:4px;color:#d4e1e7;white-space:nowrap}.sigTiny{font-size:8.5px;margin-top:3px;color:#829aa6;line-height:1.15}.sigGreen{color:#57f09a}.sigRed{color:#ff6673}.sigYellow{color:#ffe36b}.sigGray{color:#a9bac3}
.arrowKey{position:absolute;z-index:19;top:calc(50% + 74px);font-size:10px;font-weight:900;color:#d9e6ec;opacity:.82;pointer-events:none}
.arrowKey.left{left:16px}.arrowKey.right{right:16px}
#hud{border:1px solid #35566a;border-radius:8px;background:#102531;color:#d9f4ff;padding:9px 12px;font-size:12px;font-weight:800}.side{padding:10px;display:flex;flex-direction:column;gap:8px;min-height:0;overflow-y:auto;overflow-x:hidden;overscroll-behavior:contain;scrollbar-gutter:stable;touch-action:pan-y}.card{border:1px solid #263f4e;border-radius:10px;background:#08151d;padding:9px}.card h3{margin:0 0 7px;font-size:14px}.metric{display:flex;justify-content:space-between;font-size:12px;line-height:1.55;gap:12px}.metric span:first-child{color:var(--muted)}.metric b{text-align:right}.teacher{color:var(--gold)}.rear{color:var(--purple)}.bad{color:#ff6b6b}.dim{color:#8fa8b5}.fusion{color:var(--cyan)}.front{color:var(--green)}.zones{display:grid;grid-template-columns:repeat(5,1fr);gap:4px}.zone{text-align:center;border:1px solid #334b58;background:#0b1c25;border-radius:7px;padding:6px 2px;font-size:10px}.zone.on{background:#243518;border-color:#587c37;color:#a5ff75}.objList{font-family:ui-monospace,Consolas,monospace;font-size:10.5px;overflow:visible;min-height:0;flex:none}.obj{display:grid;grid-template-columns:54px 38px 1fr;gap:4px;padding:5px 2px;border-bottom:1px solid #152a35}.obj .id{font-weight:700}.badge{display:inline-block;border-radius:4px;padding:1px 4px;font-size:8.5px;margin-left:3px}.bScc{background:#7a5a00;color:#fff1a8}.bRear{background:#662276;color:#ffd6ff}.bLink{background:#174f32;color:#aaffc3}.bCam{background:#164d78;color:#d3efff}.bL1{background:#695400;color:#ffe78a}.bL2{background:#6b2c0d;color:#ffbd8f}.bRoad{background:#19485b;color:#b7f5ff}.footer{font-size:9.5px;color:#78909d;line-height:1.35}.side::-webkit-scrollbar{width:9px}.side::-webkit-scrollbar-track{background:#071219}.side::-webkit-scrollbar-thumb{background:#35566a;border-radius:8px;border:2px solid #071219}.side::-webkit-scrollbar-thumb:hover{background:#4c748a}@media(max-width:900px){.main{grid-template-columns:1fr;grid-template-rows:minmax(0,1fr) 220px}.side{display:grid;grid-template-columns:1fr 1fr 1fr;overflow:auto}.objList{grid-column:1/4}.brand{display:none}.tabs button,.range button{padding:7px 7px;font-size:10px}.top{height:52px;padding:5px}}
</style></head><body><div class="top"><div class="tabs"><button data-mode="corner" class="active">CORNER FUSED</button><button data-mode="cornerdbg">CORNER DBG</button><button data-mode="front">FRONT RADAR</button><button data-mode="stdpreview">STD FRONT</button><button data-mode="all">360 CANON</button><button data-mode="filtered">FILTERED STAGE</button><button data-mode="raw">RAW</button><button data-mode="shadow">SHADOW L1/L2</button></div><div class="range"><button data-range="short" class="active">SHORT 1:1</button><button data-range="long">LONG 1:1</button><button data-range="wide">WIDE</button><button data-range="drive">DRIVE 3-LANE</button></div><div class="spacer"></div><div class="brand"><b>G80 5-Radar (v38R2)</b><br><em>SOURCE TRACE · ROAD GATE · SIGNAL PROBE · FG6/DEC3</em></div><div id="liveRate" class="live">● TARGET 10 Hz</div><button id="hud">HUD</button><button id="fs">전체</button></div><div class="main"><div class="scene"><div id="intentBanner" class="intentBanner"><div class="intentMain">BLINKER OFF</div><div class="intentSub">DEC3 SHADOW · 좌/우 삼각형=미리보기</div><div class="intentTiny">차선변경 허가 신호가 아님</div></div><div id="signalProbe" class="signalProbe"><div><div class="tlHead"><div id="tlRed" class="tlBulb red"></div><div id="tlYellow" class="tlBulb yellow"></div><div id="tlGreen" class="tlBulb green"></div></div><div id="tlArrow" class="tlArrow">↔</div></div><div class="sigText"><div id="sigMain" class="sigMain sigGray">SIGNAL ?</div><div id="sigSub" class="sigSub">E2E probe waiting</div><div id="sigTiny" class="sigTiny">score=heuristic · direct lamp classifier 아님</div></div></div><div id="leftArrow" class="sideArrow left off"></div><div id="rightArrow" class="sideArrow right off"></div><div class="arrowKey left">LEFT</div><div class="arrowKey right">RIGHT</div><canvas id="radar"></canvas></div><div class="side"><div class="card"><h3>상태</h3><div id="summary"></div></div><div class="card"><h3>표시 기준</h3><div id="modeLegend"></div></div><div class="card"><h3>차선 점유</h3><div id="zones" class="zones"></div></div><div class="card"><h3>Teacher</h3><div id="teachers"></div></div><div id="objects" class="card objList"></div><div class="footer">v38R2 MONITOR ONLY: source-traceable Canonical360 + Corner Debug + road/lane SAFE gate + KF4 + IMM3 + FG6 + DEC3 + TURN/LATCH ARROW HUD + E2E SIGNAL PROBE. SAFE/CAUTION/BLOCKED is diagnostic only; no radarTracks/radarState publish, no CAN TX.</div></div></div><script>
const cvs=document.getElementById('radar'),ctx=cvs.getContext('2d',{alpha:false});const scene=cvs.parentElement,summary=document.getElementById('summary'),modeLegend=document.getElementById('modeLegend'),zonesEl=document.getElementById('zones'),teachers=document.getElementById('teachers'),objectsEl=document.getElementById('objects'),intentBanner=document.getElementById('intentBanner'),leftArrow=document.getElementById('leftArrow'),rightArrow=document.getElementById('rightArrow'),tlRed=document.getElementById('tlRed'),tlYellow=document.getElementById('tlYellow'),tlGreen=document.getElementById('tlGreen'),tlArrow=document.getElementById('tlArrow'),sigMain=document.getElementById('sigMain'),sigSub=document.getElementById('sigSub'),sigTiny=document.getElementById('sigTiny');let mode='corner',rangeMode='short',state=null,dirty=true,lastSideUpdate=0;let cssW=1,cssH=1,dpr=1;const bg=document.createElement('canvas'),bgc=bg.getContext('2d',{alpha:false});const C={grid:'#1b3340',lane:'#8099a6',edge:'#415661',text:'#eaf4f8',muted:'#829ba8',cyan:'#16d4e3',green:'#57d96a',gold:'#ffd34e',purple:'#cf4eff',orange:'#ff913c',gray:'#9cb1bc',camera:'#76c7ff',FRONT:'#2979ff',FL:'#00e5ff',FR:'#ff5fc8',RL:'#8cff52',RR:'#ff9f43',FC:'#5ad0ff',RC:'#c7a7ff',FILTERED:'#b388ff',PREVIEW_CONF:'#00e676',PREVIEW_CORR:'#ffd54f',PREVIEW_CAND:'#9aa7b2',RAW_A:'#00b8d4',RAW_B:'#ff6fae',RAW_F:'#448aff'};
function q(){if(rangeMode==='short')return{rear:-15,front:35,step:5,stretch:false};if(rangeMode==='long')return{rear:-50,front:100,step:10,stretch:false};if(rangeMode==='wide')return{rear:-30,front:70,step:10,stretch:true,lat:12.6,wide:true};return{rear:-12,front:40,step:5,stretch:true,lat:7.2,drive:true,perspective:true}}function mppY(){const a=q();return cssH/(a.front-a.rear)}function mppX(){const a=q();return a.stretch?cssW/(2*a.lat):mppY()}function persp(x){const a=q();if(!a.perspective)return 1;const f=Math.max(0,Number(x));return Math.max(.50,1/(1+f/55))}function X(y,x=0){return cssW/2-y*mppX()*persp(x)}function Y(x){return(q().front-x)*mppY()}function visible(x,y){const a=q(),half=a.stretch?a.lat:cssW/(2*mppX());return x>=a.rear&&x<=a.front&&y>=-half/persp(x)&&y<=half/persp(x)}function rounded(c,x,y,w,h,r,fill,stroke,lw=1){c.beginPath();c.roundRect(x,y,w,h,r);if(fill){c.fillStyle=fill;c.fill()}if(stroke){c.strokeStyle=stroke;c.lineWidth=lw;c.stroke()}}
function car(c,xm,ym,color,label,highlight=false){const a=q(),ego=(label==='G80'),cx=X(ym,xm);let cy=Y(xm),W,H;if(a.drive){const ps=persp(xm),lanePx=3.6*mppX()*ps;if(ego){W=Math.max(66,Math.min(92,lanePx*.38));H=Math.max(102,W*1.55)}else{W=Math.max(34,Math.min(82,lanePx*.34));H=Math.max(54,W*1.55)}}else if(a.wide){W=ego?Math.max(30,1.9*mppX()):26;H=ego?Math.max(56,4.8*mppY()):50}else{W=Math.max(14,1.8*mppX());H=Math.max(28,(ego?4.8:4.5)*mppY())}if(ego)cy=Y(0)+H/2;rounded(c,cx-W/2,cy-H/2,W,H,Math.min(9,W*.28),color+'44',color,a.drive?2.0:1.6);rounded(c,cx-W*.27,cy-H*.28,W*.54,H*.56,Math.min(5,W*.16),'#263c49','#76909e',.8);c.fillStyle=color;c.fillRect(cx-W*.40,cy-H*.44,W*.80,Math.max(2,H*.06));if(highlight){c.strokeStyle=C.gold;c.lineWidth=3;c.beginPath();c.arc(cx,cy,Math.max(W,H)*.62,0,Math.PI*2);c.stroke();c.fillStyle=C.gold;c.font='bold 14px Arial';c.fillText('★',cx+W*.55,cy-H*.48)}c.fillStyle=C.text;c.font=a.drive?'bold 11px Arial':'bold 10px Arial';c.fillText(label,cx+W*.62,cy-3);return[cx,cy,W,H]}
function buildBg(){bg.width=cvs.width;bg.height=cvs.height;bgc.setTransform(dpr,0,0,dpr,0,0);const g=bgc.createLinearGradient(0,0,0,cssH);g.addColorStop(0,'#0a1d28');g.addColorStop(.55,'#07141c');g.addColorStop(1,'#030a0f');bgc.fillStyle=g;bgc.fillRect(0,0,cssW,cssH);const a=q();bgc.font='11px Arial';bgc.textAlign='left';for(let x=Math.ceil(a.rear/a.step)*a.step;x<=a.front;x+=a.step){const yy=Y(x);bgc.strokeStyle=x===0?'#77909c':C.grid;bgc.lineWidth=x===0?1.5:1;bgc.beginPath();bgc.moveTo(0,yy);bgc.lineTo(cssW,yy);bgc.stroke();bgc.fillStyle=C.muted;bgc.fillText(x===0?'0m FRONT':((x>0?'+':'')+x+'m'),6,yy-4)}if(a.drive){bgc.fillStyle='rgba(255,255,255,.80)';bgc.font='bold 11px Arial';bgc.fillText('DRIVE · 3 LANES · C4 ROAD MODEL',10,17)}else if(a.wide){bgc.fillStyle='rgba(255,255,255,.72)';bgc.font='bold 10px Arial';bgc.fillText('WIDE · C4 ROAD MODEL',10,16)}else{const bar=5*mppX();bgc.strokeStyle='#d8e6ec';bgc.lineWidth=2;bgc.beginPath();bgc.moveTo(cssW-20-bar,cssH-20);bgc.lineTo(cssW-20,cssH-20);bgc.stroke();bgc.fillStyle=C.muted;bgc.fillText('5 m',cssW-20-bar/2-10,cssH-25)}}
function resize(){const r=scene.getBoundingClientRect(),nd=Math.min(window.devicePixelRatio||1,2);const nw=Math.max(2,Math.floor(r.width*nd)),nh=Math.max(2,Math.floor(r.height*nd));if(nw!==cvs.width||nh!==cvs.height){dpr=nd;cssW=r.width;cssH=r.height;cvs.width=nw;cvs.height=nh;ctx.setTransform(dpr,0,0,dpr,0,0);buildBg();dirty=true}}window.addEventListener('resize',resize);new ResizeObserver(resize).observe(scene);resize();
function poly(points,color,width=1.5,dash=[]){if(!points||points.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();let started=false;for(const p of points){const x=Number(p.x),y=Number(p.y);if(!Number.isFinite(x)||!Number.isFinite(y))continue;const sx=X(y,x),sy=Y(x);if(!started){ctx.moveTo(sx,sy);started=true}else ctx.lineTo(sx,sy)}if(started)ctx.stroke();ctx.setLineDash([])}
function roadYAtX(x){const rm=state?.road_model||{},p=rm.path||[];if(!rm.fresh||p.length<2)return null;const xmax=Number(rm.path_x_max_m??p[p.length-1].x),margin=Number(rm.path_projection_margin_m??4);if(x>xmax+margin)return null;let prev=p[0];for(let i=1;i<p.length;i++){const cur=p[i];if(Number(cur.x)>=x){const dx=Number(cur.x)-Number(prev.x);if(Math.abs(dx)<1e-6)return Number(prev.y);const t=(x-Number(prev.x))/dx;return Number(prev.y)+(Number(cur.y)-Number(prev.y))*t}prev=cur}return null}
function offsetPath(offset){const rm=state?.road_model||{},p=rm.path||[];if(p.length<2)return[];const out=[];for(let i=0;i<p.length;i++){const a=p[Math.max(0,i-1)],b=p[Math.min(p.length-1,i+1)],dx=Number(b.x)-Number(a.x),dy=Number(b.y)-Number(a.y),n=Math.hypot(dx,dy)||1;const nx=-dy/n,ny=dx/n;out.push({x:Number(p[i].x)+nx*offset,y:Number(p[i].y)+ny*offset})}return out}
function straightFallback(x0,x1,offset){return[{x:x0,y:offset},{x:x1,y:offset}]}
function drawRoad(){const a=q(),rm=state?.road_model||{},fresh=!!rm.fresh&&(rm.path||[]).length>1;const laneW=3.6,outer=laneW*1.5;const offsets=a.drive?[-outer,-laneW/2,laneW/2,outer]:[-9,-5.4,-1.8,1.8,5.4,9];if(fresh){let actualLines=(rm.lane_lines||[]).filter(ln=>Number(ln.prob||0)>=.35&&Array.isArray(ln.points)&&ln.points.length>1);let fillLeft=offsetPath(outer),fillRight=offsetPath(-outer);if(a.drive&&actualLines.length>=4){const meanY=ln=>{const pts=ln.points||[];if(!pts.length)return 0;const n=Math.min(8,pts.length);let z=0;for(let i=0;i<n;i++)z+=Number(pts[i].y)||0;return z/n};const sorted=actualLines.slice().sort((u,v)=>meanY(v)-meanY(u));fillLeft=sorted[0].points||fillLeft;fillRight=sorted[sorted.length-1].points||fillRight}if(fillLeft.length>1&&fillRight.length>1){ctx.fillStyle=a.drive?'rgba(22,212,227,.050)':'rgba(22,212,227,.036)';ctx.beginPath();for(let i=0;i<fillLeft.length;i++){const p=fillLeft[i];if(i===0)ctx.moveTo(X(p.y,p.x),Y(p.x));else ctx.lineTo(X(p.y,p.x),Y(p.x))}for(let i=fillRight.length-1;i>=0;i--){const p=fillRight[i];ctx.lineTo(X(p.y,p.x),Y(p.x))}ctx.closePath();ctx.fill()}if(!a.drive||actualLines.length<3){for(const off of offsets)poly(offsetPath(off),'rgba(92,119,133,.30)',1,[7,10])}for(const off of offsets)poly(straightFallback(a.rear,0,off),'rgba(92,119,133,.30)',1,[7,10]);for(const e of (rm.road_edges||[]))poly(e.points||[],C.edge,1,[3,8]);let actual=0;for(const ln of (rm.lane_lines||[])){const p=Number(ln.prob||0);if(p<.20)continue;actual++;ctx.globalAlpha=Math.max(.28,Math.min(1,.30+.70*p));poly(ln.points||[],C.lane,p>.6?2.2:1.35,[10,8]);ctx.globalAlpha=1}poly(rm.path||[],C.cyan,2.0,[]);if(a.drive){const labelX=Math.min(10,a.front*.30),base=roadYAtX(labelX)??0;ctx.textAlign='center';ctx.font='bold 10px Arial';ctx.fillStyle=C.muted;for(const p of [['L1',laneW],['EGO',0],['R1',-laneW]])ctx.fillText(p[0],X(base+p[1],labelX),Y(labelX)-4);ctx.textAlign='left'}ctx.fillStyle='rgba(183,245,255,.82)';ctx.font='bold 10px Arial';ctx.fillText(`C4 ${rm.curve_direction||'ROAD'} · lane ${actual}/${rm.confident_lane_lines??0} · horizon ${rm.path_x_max_m==null?'--':Number(rm.path_x_max_m).toFixed(0)+'m'} · ${rm.age_ms==null?'--':Number(rm.age_ms).toFixed(0)+'ms'}`,10,34)}else{for(const off of offsets)poly(straightFallback(a.rear,a.front,off),C.lane,1,[9,9]);ctx.fillStyle='rgba(255,107,107,.9)';ctx.font='bold 10px Arial';ctx.fillText('C4 ROAD MODEL STALE/UNAVAILABLE · straight fallback',10,34)}}
function arr(){if(!state)return[];if(mode==='corner')return state.corner_fused_objects||[];if(mode==='cornerdbg')return state.corner_debug_objects||[];if(mode==='front')return state.front_objects||[];if(mode==='stdpreview')return state.standard_front_preview||[];if(mode==='all')return state.sensor_fused_objects||state.all_fused_objects||[];if(mode==='filtered')return state.filtered_objects||[];if(mode==='shadow')return state.shadow_leads?.candidates||[];return state.raw_objects||[]}function cornerSector(o){const src=String(o.source||''),sources=o.vehicle_cluster_sources||[];const hasCorner=src.startsWith('corner')||o.corner_link_id!=null||sources.some(v=>String(v).startsWith('corner'));if(!hasCorner)return null;const s=o.sector;if(['FL','FR','RL','RR','FC','RC'].includes(s))return s;const x=Number(o.x),y=Number(o.y);if(x>.5&&y>1.2)return'FL';if(x>.5&&y<-1.2)return'FR';if(x<-.5&&y>1.2)return'RL';if(x<-.5&&y<-1.2)return'RR';if(x>.5)return'FC';if(x<-.5)return'RC';return null}
function sourceMask(o){const a=o.source_mask||o.canonical_domains||o.trace_canonical_domains||[];return Array.isArray(a)?a.map(String):[]}
function sourceMaskText(o){const a=sourceMask(o);if(!a.length)return'?';return a.map(x=>x==='FRONT'?'F':x==='CAMERA'?'CAM':x==='REAR_TEACHER'?'RT':x).join('+')}
function displayState(o){return String(o.display_state||'MEASURED').toUpperCase()}
function modeColor(o){
  // V35: color meaning is mode-specific. Evidence (SCC/CAM/L1/L2) is shown
  // with rings/badges and never replaces the sensor/stage base color.
  if(mode==='cornerdbg'){const sec=cornerSector(o);return sec&&C[sec]?C[sec]:C.gray;}
  if(mode==='front')return C.FRONT;
  if(mode==='stdpreview'){
    const q=String(o.preview_quality||'candidate').toLowerCase();
    if(q.includes('confirm'))return C.PREVIEW_CONF;
    if(q.includes('corrob')||q.includes('reference')||q.includes('camera'))return C.PREVIEW_CORR;
    return C.PREVIEW_CAND;
  }
  if(mode==='filtered')return C.FILTERED;
  if(mode==='raw'){
    const s=String(o.source||'').toLowerCase();
    if(s.includes('corner_a')||s.includes('a_241'))return C.RAW_A;
    if(s.includes('corner_b')||s.includes('b_279'))return C.RAW_B;
    if(s.includes('front')||s.includes('fr_cmr'))return C.RAW_F;
    return C.gray;
  }
  if(mode==='shadow'){
    if(o.shadow_role==='L1')return C.gold;
    if(o.shadow_role==='L2')return C.orange;
    return '#70838d';
  }
  if(mode==='all'){
    const d=String(o.canonical_primary_domain||'').toUpperCase();
    if(d==='FRONT')return C.FRONT;
    if(d==='CAMERA')return C.camera;
    if(C[d])return C[d];
    if(d==='CORNER'){const sec=cornerSector(o);return sec&&C[sec]?C[sec]:C.gray}
    return C.gray;
  }
  if(mode==='corner'){
    const sec=cornerSector(o);return sec&&C[sec]?C[sec]:C.gray;
  }
  return C.gray;
}
function modeLegendHtml(){
  if(mode==='corner')return `<b>CORNER FUSED</b><br><span style="color:${C.FL}">■ FL</span> <span style="color:${C.FR}">■ FR</span> <span style="color:${C.RL}">■ RL</span> <span style="color:${C.RR}">■ RR</span><br><span class="dim">확정 corner-fused만 · 가능하면 360과 동일 Vxxxx 표시</span>`;
  if(mode==='cornerdbg')return `<b>CORNER DBG</b><br><span style="color:${C.FL}">■ fused=실선</span> <span class="dim">candidate=점선/반투명</span><br><span class="dim">filtered corner24 후보까지 표시 · 360에만 보이는 객체의 corner 근거 확인용</span>`;
  if(mode==='front')return `<b style="color:${C.FRONT}">FRONT RADAR</b><br><span class="dim">전방 radar/reference object만 표시 · 모두 cobalt blue</span>`;
  if(mode==='stdpreview')return `<b>STD FRONT PREVIEW</b><br><span style="color:${C.PREVIEW_CONF}">■ confirmed</span> <span style="color:${C.PREVIEW_CORR}">■ corroborated</span> <span style="color:${C.PREVIEW_CAND}">■ candidate</span><br><span class="dim">향후 standard RadarPoint 후보 단계</span>`;
  if(mode==='all')return `<b>360 CANON</b><br><span style="color:${C.FRONT}">■ FRONT</span> <span style="color:${C.FL}">■ FL</span> <span style="color:${C.FR}">■ FR</span> <span style="color:${C.RL}">■ RL</span> <span style="color:${C.RR}">■ RR</span> <span style="color:${C.camera}">■ CAMERA</span><br><span class="dim">색=현재 primary source · [F/FL/.../CAM]=source mask · CONF/MEAS/PRED/STALE 표시</span>`;
  if(mode==='filtered')return `<b style="color:${C.FILTERED}">FILTERED STAGE</b><br><span class="dim">validity/filter 출력 · Vxxxx trace가 있으면 동일 canonical ID 사용 · 원래 key는 목록에 LOCAL로 표시</span>`;
  if(mode==='raw')return `<b>RAW DECODE</b><br><span style="color:${C.RAW_A}">■ Bank A</span> <span style="color:${C.RAW_B}">■ Bank B</span> <span style="color:${C.RAW_F}">■ Front raw/ref</span><br><span class="dim">fusion/dedup 전 진단용 raw stream</span>`;
  return `<b>SHADOW L1/L2</b><br><span style="color:${C.gold}">■ L1</span> <span style="color:${C.orange}">■ L2</span> <span class="dim">■ other candidate</span><br><span class="dim">lead 역할 색 · sensor evidence는 CAM/SCC badge/ring</span>`;
}
function drawTrajectory(o,col){const pts=o.kalman_trajectory||[];if(!o.kalman_valid||!pts.length)return;const x0=Number(o.kf_x??o.x),y0=Number(o.kf_y??o.y);if(!Number.isFinite(x0)||!Number.isFinite(y0))return;ctx.save();ctx.globalAlpha=o.kf_cutin_candidate?.95:.62;ctx.strokeStyle=col;ctx.lineWidth=o.kf_cutin_candidate?3:1.8;ctx.setLineDash([5,5]);ctx.beginPath();ctx.moveTo(X(y0,x0),Y(x0));for(const p of pts){const x=Number(p.x),y=Number(p.y);if(!Number.isFinite(x)||!Number.isFinite(y))continue;if(visible(x,y))ctx.lineTo(X(y,x),Y(x))}ctx.stroke();ctx.setLineDash([]);for(const p of pts){const x=Number(p.x),y=Number(p.y),tt=Number(p.t);if(!Number.isFinite(x)||!Number.isFinite(y)||!visible(x,y))continue;ctx.beginPath();ctx.arc(X(y,x),Y(x),tt>=2?3.2:2.4,0,Math.PI*2);ctx.fillStyle=col;ctx.fill();if(rangeMode==='drive'&&(Math.abs(tt-1)<.01||Math.abs(tt-2)<.01)){ctx.font='bold 8px Arial';ctx.fillStyle=col;ctx.fillText(`${tt.toFixed(0)}s`,X(y,x)+4,Y(x)-4)}}ctx.restore()}
function drawTeacherMarkers(){if(!state)return;const rear=state.teacher_rear||[];for(const t of rear){const d=Number(t.distance_candidate_m);if(!Number.isFinite(d)||d<=0)continue;const y=t.sector==='LR'?3.6:-3.6,x=-d;if(!visible(x,y))continue;const xx=X(y,x),yy=Y(x),ok=!!t.teacher_usable;ctx.strokeStyle=ok?C.purple:'#76577e';ctx.lineWidth=ok?2.5:1.4;ctx.beginPath();ctx.moveTo(xx-7,yy);ctx.lineTo(xx+7,yy);ctx.moveTo(xx,yy-7);ctx.lineTo(xx,yy+7);ctx.stroke();ctx.fillStyle=ok?C.purple:'#8d7195';ctx.font='bold 9px Arial';ctx.fillText(`${t.sector} T ${d.toFixed(1)}m S${t.status_raw??'?'}`,xx+9,yy-8)}const s=state.scc_teacher||{};if(s.distance_m!=null){const x=Number(s.distance_m),py=roadYAtX(x),y=Number.isFinite(py)?py:0;if(Number.isFinite(x)&&visible(x,y)){const xx=X(y,x),yy=Y(x),ok=!!s.teacher_usable;ctx.strokeStyle=ok?C.gold:C.orange;ctx.lineWidth=ok?3:1.5;ctx.beginPath();ctx.moveTo(xx-9,yy);ctx.lineTo(xx+9,yy);ctx.moveTo(xx,yy-9);ctx.lineTo(xx,yy+9);ctx.stroke();ctx.fillStyle=ok?C.gold:C.orange;ctx.font='bold 10px Arial';ctx.fillText(`SCC T B${s.bus??'?'} ${x.toFixed(1)}m${Number.isFinite(py)?' · PATH':''}`,xx+12,yy-9)}}}
function draw(){if(!dirty)return;dirty=false;ctx.drawImage(bg,0,0,bg.width,bg.height,0,0,cssW,cssH);drawRoad();car(ctx,0,0,'#f3f7f8','G80');for(const o of arr()){let x=Number(o.x),y=Number(o.y);const ds=displayState(o);if(mode==='all'&&ds==='PREDICTED'&&Number.isFinite(Number(o.kf_x))&&Number.isFinite(Number(o.kf_y))){x=Number(o.kf_x);y=Number(o.kf_y)}if(!Number.isFinite(x)||!Number.isFinite(y)||!visible(x,y))continue;const col=modeColor(o);ctx.save();let ghost=false;if(mode==='all'){if(ds==='STALE'){ctx.globalAlpha=.30;ctx.setLineDash([7,5]);ghost=true}else if(ds==='PREDICTED'){ctx.globalAlpha=.52;ctx.setLineDash([5,5]);ghost=true}else if(ds==='MEASURED'){ctx.globalAlpha=.78}else ctx.globalAlpha=1}else if(mode==='cornerdbg'&&o.corner_debug_role==='CANDIDATE'){ctx.globalAlpha=.40;ctx.setLineDash([5,5]);ghost=true}else if(o.trace_unmatched){ctx.globalAlpha=.55;ctx.setLineDash([4,4]);ghost=true}if(o.kalman_valid&&(rangeMode==='drive'||mode==='all'||mode==='corner'||mode==='cornerdbg'||mode==='front'||mode==='shadow'))drawTrajectory(o,col);const lane=o.road_lane?` ${String(o.road_lane).toUpperCase()}`:(o.road_lane_source==='c4_path_out_of_range'?' PATH?':''),dom=(mode==='all'&&o.canonical_primary_domain)?` ${String(o.canonical_primary_domain)}`:'',baseId=(o.canonical_key||o.vehicle_key||o.key||'?'),role=(mode==='cornerdbg'&&o.corner_debug_role==='CANDIDATE')?' CAND':'',id=baseId+dom+role+(mode==='corner'&&(o.sector||o.front_sector)?' '+(o.sector||o.front_sector):'')+(rangeMode==='drive'?lane:'');const p=car(ctx,x,y,col,id,!!o.scc_teacher_confirmed);ctx.setLineDash([]);ctx.globalAlpha=1;const src=sourceMaskText(o);if(mode==='all'||mode==='corner'||mode==='cornerdbg'||mode==='front'||mode==='filtered'){ctx.fillStyle=col;ctx.font='bold 9px Arial';ctx.fillText(`[${src}] ${ds}${o.source_age_ms==null?'':` ${Number(o.source_age_ms).toFixed(0)}ms`}`,p[0]+p[2]*.62,p[1]-18)}if(o.kf_cutin_candidate){ctx.strokeStyle=C.gold;ctx.lineWidth=3;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*.95,0,Math.PI*2);ctx.stroke();ctx.fillStyle=C.gold;ctx.font='bold 9px Arial';ctx.fillText(`KF CUT ${o.kf_ttlc_s==null?'':Number(o.kf_ttlc_s).toFixed(1)+'s'}`,p[0]+p[2]*.62,p[1]-30)}if(o.teacher_match){ctx.strokeStyle=C.purple;ctx.lineWidth=3;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*.70,0,Math.PI*2);ctx.stroke()}if(o.front_link){ctx.fillStyle=C.FRONT;ctx.font='bold 10px Arial';ctx.fillText('+FRONT',p[0]+p[2]*.62,p[1]+12)}if(o.camera_confirmed&&o.source!=='c4_camera'){ctx.strokeStyle=C.camera;ctx.lineWidth=2.2;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*.82,0,Math.PI*2);ctx.stroke();ctx.fillStyle=C.camera;ctx.font='bold 9px Arial';ctx.fillText('CAM',p[0]+p[2]*.62,p[1]+24)}if(o.shadow_role){const rc=o.shadow_role==='L1'?C.gold:C.orange;ctx.strokeStyle=rc;ctx.lineWidth=3;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*1.06,0,Math.PI*2);ctx.stroke();ctx.fillStyle=rc;ctx.font='bold 10px Arial';ctx.fillText(o.shadow_role,p[0]+p[2]*.62,p[1]-16)}if(o.vx!=null){ctx.fillStyle=col;ctx.font='10px Arial';ctx.fillText(`${x.toFixed(1)}m ${Number(o.vx).toFixed(1)}m/s`,p[0]+p[2]*.62,p[1]+10)}if((mode==='shadow'||rangeMode==='drive')&&o.road_d!=null){ctx.fillStyle=C.muted;ctx.font='9px Arial';ctx.fillText(`d ${Number(o.road_d).toFixed(2)}m`,p[0]+p[2]*.62,p[1]+22)}ctx.restore()}drawTeacherMarkers()}
function reasonLabel(x){const m={'front_gap<=5m':'FRONT GAP','rear_gap<=5m':'REAR GAP','boundary<=2m':'BOUNDARY','TTC<=3s':'TTC','stable_incoming_near':'INCOMING','front_gap<=12m':'FRONT GAP','rear_gap<=12m':'REAR GAP','boundary<=5m':'BOUNDARY','TTC<=5s':'TTC','front_brake_scenario<=8m':'FRONT BRAKE?','ego_brake_rear<=8m':'EGO BRAKE?','stable_incoming':'INCOMING?','possible_incoming_near':'INCOMING?','display_hysteresis':'HOLD','lane_change_commit_hold':'COMMIT HOLD','lane_change_rebase':'REBASE','intersection_turn_context':'TURN','commit_hold_hard_ttc_override':'TTC OVERRIDE','target_lane_absent':'NO TARGET LANE','target_lane_unconfirmed':'ROAD/LANE UNCERTAIN'};return m[x]||String(x||'').replaceAll('_',' ').toUpperCase()}
function arrowClass(label){const s=String(label||'');if(s.startsWith('SAFE'))return'safe';if(s.startsWith('DANGER'))return'danger';if(s.startsWith('CHECK'))return'check';if(s.startsWith('TURN'))return'turn';return'off'}
function updatePreArrows(){if(!state||!leftArrow||!rightArrow)return;const fg=state.future_gap||{},di=fg.driver_intent||{};const ll=fg.left?.decision?.label||'--',rr=fg.right?.decision?.label||'--';let lc=arrowClass(ll),rc=arrowClass(rr);if(di.active){if(di.maneuver_context==='TURN'){if(di.side==='left')lc='turn';if(di.side==='right')rc='turn'}else{const ac=arrowClass(di.label||'CHECK ?');if(di.side==='left')lc=ac;if(di.side==='right')rc=ac}}leftArrow.className=`sideArrow left ${lc}${di.active&&di.side==='left'?' active':''}${di.active&&di.side==='left'&&di.phase==='COMMIT_HOLD'?' pulse':''}`;rightArrow.className=`sideArrow right ${rc}${di.active&&di.side==='right'?' active':''}${di.active&&di.side==='right'&&di.phase==='COMMIT_HOLD'?' pulse':''}`}
function updateSignalProbe(){if(!state||!sigMain)return;const t=state.traffic_signal_probe||{},st=t.state||'UNKNOWN',go=Math.round(100*Number(t.go_score||0)),stop=Math.round(100*Number(t.stop_score||0));tlRed.classList.remove('on');tlYellow.classList.remove('on');tlGreen.classList.remove('on');tlArrow.classList.remove('on');sigMain.className='sigMain sigGray';if(st==='GREEN_GO'){tlGreen.classList.add('on');sigMain.className='sigMain sigGreen';sigMain.textContent=`GREEN OK · ${go}%*`}else if(st==='RED_STOP_INFERRED'){tlRed.classList.add('on');sigMain.className='sigMain sigRed';sigMain.textContent=`RED/STOP ? · ${stop}%*`}else if(st==='WAIT_LEAD'){tlYellow.classList.add('on');sigMain.className='sigMain sigYellow';sigMain.textContent='LEAD AHEAD'}else if(st==='WATCH'){tlYellow.classList.add('on');sigMain.className='sigMain sigYellow';sigMain.textContent=`SIGNAL WATCH · G${go}/S${stop}`}else if(st==='DRIVING'){sigMain.textContent='DRIVING · SIGNAL MONITOR'}else{sigMain.textContent='SIGNAL ? · MODEL STALE'}const td=t.turn_direction||'none';if(td==='left'){tlArrow.textContent='←';tlArrow.classList.add('on')}else if(td==='right'){tlArrow.textContent='→';tlArrow.classList.add('on')}else{tlArrow.textContent='↔'}const h=t.path_horizon_m==null?'--':Number(t.path_horizon_m).toFixed(1);sigSub.textContent=`path ${h}m · stop ${t.should_stop?1:0} · SP green ${t.sunnypilot_green_alert?1:0} · ${td==='none'?'TURN --':('TURN '+td.toUpperCase())}`;sigTiny.textContent=`* heuristic score, not probability · model confidence=${t.model_confidence||'--'} ≠ lamp color`;}
function updateIntent(){updateSignalProbe();if(!state||!intentBanner)return;const fg=state.future_gap||{},di=fg.driver_intent||{},active=!!di.active,side=di.side||'',label=di.label||'STANDBY';updatePreArrows();intentBanner.className='intentBanner';if(!active){intentBanner.innerHTML=`<div class="intentMain">PREVIEW</div><div class="intentSub">LEFT ${fg.left?.decision?.label||'--'} · RIGHT ${fg.right?.decision?.label||'--'}</div><div class="intentTiny">삼각형 색 = 미리보기 · DEC3 SHADOW / 제어 아님</div>`;return}const arrow=side==='left'?'←':'→',name=side==='left'?'LEFT':'RIGHT';if(di.maneuver_context==='TURN'||di.state==='TURN'){intentBanner.classList.add('active','turn');intentBanner.innerHTML=`<div class="intentMain"><span class="arrow">${arrow}</span> ${name} · TURN</div><div class="intentSub">교차로/저속 회전으로 판단 · lane-change SAFE/DANGER 숨김</div><div class="intentTiny">v ${(Number(fg.ego?.v_ego_mps||0)*3.6).toFixed(1)} km/h</div>`;return}let cls=label.startsWith('DANGER')?'danger':(label.startsWith('SAFE')?'safe':'check');intentBanner.classList.add('active',cls);const f=di.front_clearance_m==null?'--':Number(di.front_clearance_m).toFixed(1),r=di.rear_clearance_m==null?'--':Number(di.rear_clearance_m).toFixed(1);const rs=(di.reasons||[]).slice(0,3).map(reasonLabel).join(' · ')||'NO ACTIVE REASON';const ph=di.phase||'PRECHECK',hold=di.hold_remaining_s==null?'':` · HOLD ${Number(di.hold_remaining_s).toFixed(1)}s`;intentBanner.innerHTML=`<div class="intentMain"><span class="arrow">${arrow}</span> ${name} · ${label}</div><div class="intentSub">${ph}${hold} · F ${f} m · R ${r} m · ${rs}</div><div class="intentTiny">SHADOW DRIVER COMPARISON ONLY · 실제 차선변경 허가 신호가 아님</div>`}
function side(){if(!state)return;updateIntent();const now=performance.now();if(now-lastSideUpdate<180)return;lastSideUpdate=now;const cs=state.corner_fused_objects||[],fs=state.front_objects||[],sp=state.standard_front_preview||[],sps=state.standard_front_preview_stats||{},as=state.corner_front_associations||[],cams=state.camera_leads||[],cm=state.camera_fusion_matches||[],sh=state.shadow_leads||{},ss=sh.stats||{},lg=state.shadow_logger||{},raws=state.raw_objects||[],cst=state.camera_fusion_stats||{},cfs=state.corner_fusion_stats||{},t=state.scc_teacher||{},m=state.scc_front_match||{},diag=state.diagnostics||{},rm=state.road_model||{},cts=state.canonical_tracker_stats||{},ks=state.kalman_motion_stats||{},ims=state.imm_motion_stats||{},fg=state.future_gap||{},tl=state.traffic_signal_probe||{},ps=state.performance_stats||{},vcs=state.view_consistency_stats||{};const pi=Number(ps.publish_interval_ms);const hz=(Number.isFinite(pi)&&pi>1)?(1000/pi):0;if(window.liveRate)liveRate.textContent=hz>0?`● LIVE ${hz.toFixed(1)} Hz`:'● TARGET 10 Hz';const sccRx=(t.bus!==undefined)?`RX bus${t.bus}`:'NO 0x1A0 RX';modeLegend.innerHTML=modeLegendHtml();const py40=rm.path_y_40m==null?'--':Number(rm.path_y_40m).toFixed(2)+'m';summary.innerHTML=`<div class="metric"><span>Mode</span><b>${mode.toUpperCase()} / ${rangeMode==='wide'?'WIDE':(rangeMode==='drive'?'DRIVE 3-LANE':rangeMode.toUpperCase()+' 1:1')}</b></div><div class="metric"><span>Runtime</span><b class="${state.runtime_mismatch?'bad':'fusion'}">${state.runtime_mismatch?'VERSION MISMATCH':('v'+(state.runtime_versions?.build??'?')+' / L'+(state.runtime_versions?.logger??'?')+' / KF'+(state.runtime_versions?.kalman_api??'?')+' / IMM'+(state.runtime_versions?.imm_api??'?')+' / FG'+(state.runtime_versions?.future_gap_api??'?')+' / UDP'+(state.runtime_versions?.android_protocol??'?'))}</b></div><div class="metric"><span>Build tag</span><b class="fusion">${state.runtime_versions?.tag??'--'}</b></div><div class="metric"><span>Signal probe</span><b class="${tl.state==='GREEN_GO'?'fusion':(tl.state==='RED_STOP_INFERRED'?'bad':'teacher')}">${tl.label??'--'}</b></div><div class="metric"><span>GO / STOP score*</span><b>${Math.round(100*Number(tl.go_score||0))}% / ${Math.round(100*Number(tl.stop_score||0))}%</b></div><div class="metric"><span>Path / shouldStop</span><b>${tl.path_horizon_m==null?'--':Number(tl.path_horizon_m).toFixed(1)+'m'} / ${tl.should_stop?'YES':'NO'}</b></div><div class="metric"><span>SP green / turn path</span><b>${tl.sunnypilot_green_alert?'ALERT':'--'} / ${(tl.turn_direction||'none').toUpperCase()}</b></div><div class="metric"><span>x=0 reference</span><b class="fusion">FRONT BUMPER</b></div><div class="metric"><span>C4 road model</span><b class="${rm.fresh?'fusion':'bad'}">${rm.fresh?'FRESH':'STALE'} ${rm.age_ms==null?'':Number(rm.age_ms).toFixed(0)+'ms'}</b></div><div class="metric"><span>Curve / path@40m</span><b class="fusion">${rm.curve_direction||'--'} / ${py40}</b></div><div class="metric"><span>Lane lines / horizon</span><b>${rm.confident_lane_lines??0}/${(rm.lane_lines||[]).length} / ${rm.path_x_max_m==null?'--':Number(rm.path_x_max_m).toFixed(0)+'m'}</b></div><div class="metric"><span>Canonical 360</span><b class="fusion">${cts.visible_tracks??0} / active ${cts.active_tracks??0}</b></div><div class="metric"><span>Trace miss F/Cf/Cany</span><b class="${((vcs.canonical_front_without_front_view||0)+(vcs.canonical_corner_without_fused_view||0)+(vcs.canonical_corner_without_corner_view||0))?'bad':'fusion'}">${vcs.canonical_front_without_front_view??0} / ${vcs.canonical_corner_without_fused_view??0} / ${vcs.canonical_corner_without_corner_view??0}</b></div><div class="metric"><span>Trace mapped F/C</span><b>${vcs.front_view_traced??0}/${vcs.corner_view_traced??0} · cand ${vcs.corner_candidates_traced??0}</b></div><div class="metric"><span>Canon match/new</span><b>${cts.matched_existing??0} / ${cts.new_tracks??0}</b></div><div class="metric"><span>Canon alias/kin</span><b>${cts.alias_matches??0} / ${cts.kinematic_matches??0}</b></div><div class="metric"><span>Reacq / handoff</span><b class="fusion">${cts.reacquired_tracks??0} / ${cts.source_handoffs??0}</b></div><div class="metric"><span>Continuity / ambiguous</span><b>${Number(cts.continuity_ratio??0).toFixed(2)} / ${cts.ambiguous_objects??0}</b></div><div class="metric"><span>Kalman tracks</span><b class="fusion">${ks.visible_tracks??0} / active ${ks.active_tracks??0}</b></div><div class="metric"><span>Frenet / KF cand/conf</span><b class="${(ks.cutin_confirmed||0)>0?'teacher':'fusion'}">${ks.frenet_valid_tracks??0} / ${ks.cutin_candidates??0}/${ks.cutin_confirmed??0}</b></div><div class="metric"><span>Low-speed lateral</span><b>${ks.low_speed_lateral_candidates??0}</b></div><div class="metric"><span>KF pred limited</span><b>${ks.lateral_prediction_limited_tracks??0}</b></div><div class="metric"><span>KF confident / unstable</span><b>${ks.motion_confident_tracks??0} / ${ks.lateral_unstable_tracks??0}</b></div><div class="metric"><span>KF ID/reset suspect</span><b class="${(ks.canonical_key_mismatch_tracks||ks.kf_reset_suspect_tracks)?'bad':'fusion'}">${ks.canonical_key_mismatch_tracks??0} / ${ks.kf_reset_suspect_tracks??0}</b></div><div class="metric"><span>KF dormant preserved</span><b>${ks.dormant_preserved_tracks??0}</b></div><div class="metric"><span>IMM CV/CA/MAN</span><b class="fusion">${ims.dominant_cv??0}/${ims.dominant_ca??0}/${ims.dominant_maneuver??0}</b></div><div class="metric"><span>IMM man/reset/reinit</span><b class="${(ims.reset_suspect_tracks||0)>0?'bad':'fusion'}">${ims.maneuver_candidates??0} / ${ims.reset_suspect_tracks??0} / ${ims.expected_reinitializations??0}</b></div><div class="metric"><span>IMM ROI / tick</span><b>${ims.interaction_relevant_tracks??0}/${ims.visible_tracks??0} · ${ims.evaluated_this_cycle?'EVAL':'CACHE'} ${Number(ims.eval_age_ms??0).toFixed(0)}ms</b></div><div class="metric"><span>FG Left F/R</span><b class="fusion">${fg.left?.min_front_clearance_during_ego_overlap_m==null?'--':Number(fg.left.min_front_clearance_during_ego_overlap_m).toFixed(1)+'m'} / ${fg.left?.min_rear_clearance_during_ego_overlap_m==null?'--':Number(fg.left.min_rear_clearance_during_ego_overlap_m).toFixed(1)+'m'}</b></div><div class="metric"><span>FG Right F/R</span><b class="fusion">${fg.right?.min_front_clearance_during_ego_overlap_m==null?'--':Number(fg.right.min_front_clearance_during_ego_overlap_m).toFixed(1)+'m'} / ${fg.right?.min_rear_clearance_during_ego_overlap_m==null?'--':Number(fg.right.min_rear_clearance_during_ego_overlap_m).toFixed(1)+'m'}</b></div><div class="metric"><span>Incoming raw L/R</span><b>${fg.left?.incoming_count??0} / ${fg.right?.incoming_count??0}</b></div><div class="metric"><span>Incoming stable L/R</span><b class="teacher">${fg.left?.stable_incoming_count??0} / ${fg.right?.stable_incoming_count??0}</b></div><div class="metric"><span>Incoming possible L/R</span><b>${fg.left?.possible_incoming_count??0} / ${fg.right?.possible_incoming_count??0}</b></div><div class="metric"><span>Lane gate L/R</span><b class="${((fg.left?.lane_availability?.status||'UNCERTAIN')==='CONFIRMED'&&(fg.right?.lane_availability?.status||'UNCERTAIN')==='CONFIRMED')?'fusion':'teacher'}">${fg.left?.lane_availability?.status??'--'} / ${fg.right?.lane_availability?.status??'--'}</b></div><div class="metric"><span>Road edge L/R</span><b>${fg.left?.lane_availability?.edge_extent_m==null?'--':Number(fg.left.lane_availability.edge_extent_m).toFixed(1)+'m'} / ${fg.right?.lane_availability?.edge_extent_m==null?'--':Number(fg.right.lane_availability.edge_extent_m).toFixed(1)+'m'}</b></div><div class="metric"><span>DEC3 L/R</span><b>${fg.left?.decision?.state??'--'} / ${fg.right?.decision?.state??'--'}</b></div><div class="metric"><span>Intent / phase</span><b class="teacher">${fg.driver_intent?.maneuver_context??'STANDBY'} / ${fg.driver_intent?.phase??'--'}</b></div><div class="metric"><span>Steer / curve</span><b>${Number(fg.driver_intent?.steering_angle_deg??0).toFixed(0)}° / ${fg.driver_intent?.road_curve_direction??'--'}</b></div><div class="metric"><span>Brake F L/R</span><b>${fg.left?.scenarios?.front_target_brake?.min_clearance_m==null?'--':Number(fg.left.scenarios.front_target_brake.min_clearance_m).toFixed(1)} / ${fg.right?.scenarios?.front_target_brake?.min_clearance_m==null?'--':Number(fg.right.scenarios.front_target_brake.min_clearance_m).toFixed(1)} m</b></div><div class="metric"><span>EgoBrake R L/R</span><b>${fg.left?.scenarios?.rear_ego_brake?.min_clearance_m==null?'--':Number(fg.left.scenarios.rear_ego_brake.min_clearance_m).toFixed(1)} / ${fg.right?.scenarios?.rear_ego_brake?.min_clearance_m==null?'--':Number(fg.right.scenarios.rear_ego_brake.min_clearance_m).toFixed(1)} m</b></div><div class="metric"><span>Future Gap</span><b class="teacher">FG6 + DEC3 + ROAD GATE</b></div><div class="metric"><span>Loop / UI</span><b>${Number(ps.processing_ms??0).toFixed(1)}ms / ${Number(ps.ui_json_kb??0).toFixed(1)}KB</b></div><div class="metric"><span>Core / KF / IMM</span><b>${Number(ps.stage_ms?.core_fusion??0).toFixed(1)} / ${Number(ps.stage_ms?.kf??0).toFixed(1)} / ${Number(ps.stage_ms?.imm??0).toFixed(1)} ms</b></div><div class="metric"><span>Corner cand→local→fused</span><b class="fusion">${cfs.input_corner_points??0}→${cfs.after_same_sensor_local_fusion??0}→${cfs.after_cross_corner_fusion??cs.length}</b></div><div class="metric"><span>Vehicle dedup</span><b>${cst.vehicle_objects_before??0}→${cst.vehicle_objects_after??0} / merge ${cst.vehicle_duplicates_merged??0}</b></div><div class="metric"><span>Near-speed gate</span><b>${Number(cst.near_same_source_dist_m??2.2).toFixed(1)}m/${Number(cst.near_same_source_dv_mps??1.5).toFixed(1)} · X ${Number(cst.near_cross_source_dist_m??3.5).toFixed(1)}m/${Number(cst.near_cross_source_dv_mps??2.0).toFixed(1)}</b></div><div class="metric"><span>Front ref tracks</span><b class="front">${fs.length}</b></div><div class="metric"><span>STD preview</span><b class="teacher">${sp.length}</b></div><div class="metric"><span>G1 raw→dedup / invalid</span><b>${sps.group1_fresh_before_dedup??0}→${sps.group1_after_dedup??0} / ${sps.group1_invalid_sentinel_rejected??0}</b></div><div class="metric"><span>Preview C/R/Cand</span><b>${sps.confirmed_count??0}/${sps.corroborated_count??0}/${sps.candidate_count??0}</b></div><div class="metric"><span>Preview ref/CAM</span><b>${sps.reference_match_count??0}/${sps.camera_match_count??0}</b></div><div class="metric"><span>Corner↔Front</span><b>${as.length}</b></div><div class="metric"><span>C4 camera leads</span><b style="color:${C.camera}">${cams.length}</b></div><div class="metric"><span>CAM↔Radar</span><b style="color:${C.camera}">${cm.length}</b></div><div class="metric"><span>RAW objects</span><b style="color:#3e9fff">${raws.length}</b></div><div class="metric"><span>Shadow L1/L2</span><b>${sh.leadOne?.status?(sh.leadOne.key||'L1'):'--'} / ${sh.leadTwo?.status?(sh.leadTwo.key||'L2'):'--'}</b></div><div class="metric"><span>CONTROL</span><b class="bad">MONITOR ONLY</b></div>`;const names=rangeMode==='drive'?['left1','ego','right1']:['left2','left1','ego','right1','right2'];zonesEl.innerHTML=names.map(n=>`<div class="zone ${(state.zones?.[n]?.occupied)?'on':''}">${n.replace('left','L').replace('right','R').toUpperCase()}</div>`).join('');const rearAll=state.teacher_rear||[];const rearText=rearAll.length?rearAll.map(x=>`${x.sector} ${Number(x.distance_candidate_m).toFixed(1)}m S${x.status_raw}${x.teacher_usable?'✓':''}`).join(' / '):'NO 0x1EA RX';const rawDist=(t.distance_m!==undefined)?Number(t.distance_m).toFixed(1)+'m':'--';const rawV=(t.rel_speed_mps!==undefined)?Number(t.rel_speed_mps).toFixed(1)+'m/s':'--';const flags=(t.bus!==undefined)?`B${t.bus} M${t.main_mode_acc??'?'} A${t.acc_mode??'?'} V${t.obj_valid_raw??'?'} S${t.scc_obj_sta??'?'}`:'--';const rms=state.rear_teacher_match_stats||{},rearMatch=(rms.matches||[]).map(x=>`${x.sector||'?'} ${x.key||''} Δ${Number(x.error_m??0).toFixed(2)}m`).join(' / ')||'--';teachers.innerHTML=`<div class="metric"><span>Rear 0x1EA</span><b class="${rearAll.length?'rear':'bad'}">${rearText}</b></div><div class="metric"><span>Rear match</span><b class="rear">${rearMatch}</b></div><div class="metric"><span>SCC 0x1A0</span><b class="${t.bus!==undefined?'teacher':'bad'}">${sccRx}</b></div><div class="metric"><span>SCC raw</span><b>${rawDist} / ${rawV}</b></div><div class="metric"><span>flags</span><b>${flags}</b></div><div class="metric"><span>strict usable</span><b class="${t.teacher_usable?'teacher':'dim'}">${t.teacher_usable?'YES':'NO'}</b></div><div class="metric"><span>match</span><b class="teacher">${m.confirmed?'CONF '+(m.front_key||''):(m.matched?'SEARCH':'--')}</b></div><div style="margin-top:7px;font-size:10px"><b style="color:${C.FRONT}">■ FRONT</b> &nbsp;<b style="color:${C.FL}">■ FL</b> &nbsp;<b style="color:${C.FR}">■ FR</b> &nbsp;<b style="color:${C.RL}">■ RL</b> &nbsp;<b style="color:${C.RR}">■ RR</b><br><span class="dim">FRONT=전방 radar 고유색 · corner 색상은 위치 sector 추정(센서 ownership 확정 아님)</span></div><hr style="border:0;border-top:1px solid #24404f;margin:8px 0"><div class="metric"><span>Shadow leadOne</span><b class="teacher">${sh.leadOne?.status?`${sh.leadOne.key} ${Number(sh.leadOne.dRel).toFixed(1)}m`:'--'}</b></div><div class="metric"><span>L1 handoff</span><b class="${ss.leadOne_strong_handoff?'teacher':'dim'}">${ss.leadOne_strong_handoff?`${ss.leadOne_handoff_from}→${ss.leadOne_handoff_to} Δ${Number(ss.leadOne_handoff_gain_m??0).toFixed(1)}m`:(ss.leadOne_takeover_pending_key?`PEND ${ss.leadOne_takeover_pending_key}`:'--')}</b></div><div class="metric"><span>Shadow leadTwo</span><b style="color:${C.orange}">${sh.leadTwo?.status?`${sh.leadTwo.key} ${Number(sh.leadTwo.dRel).toFixed(1)}m`:'--'}</b></div><div class="metric"><span>path age</span><b>${ss.path_age_ms==null?'--':Number(ss.path_age_ms).toFixed(0)+'ms'}</b></div><div class="metric"><span>Shadow log</span><b style="color:${lg.enabled?C.green:C.muted}">${lg.enabled?`ON ${lg.records||0}`:'OFF'}</b></div><div class="metric"><span>radarTracks TX</span><b class="bad">OFF</b></div>`;objectsEl.innerHTML=arr().slice().sort((a,b)=>Number(a.x)-Number(b.x)).slice(0,24).map(o=>`<div class="obj"><span class="id" style="color:${modeColor(o)}">${o.canonical_key||o.vehicle_key||o.key||'?'}</span><span>${o.sector||o.front_sector||''}</span><span>${Number(o.x).toFixed(1)}m ${o.vx==null?'':Number(o.vx).toFixed(1)+'m/s'}<span class="badge bLink">SRC ${sourceMaskText(o)}</span><span class="badge ${displayState(o)==='STALE'?'bL2':(displayState(o)==='PREDICTED'?'bScc':'bRoad')}">${displayState(o)}</span>${o.trace_local_key&&o.trace_local_key!==(o.canonical_key||'')?`<span class="badge" style="background:#26333b;color:#aabcc6">LOCAL ${o.trace_local_key}</span>`:''}${o.trace_match_method?`<span class="badge bCam">TRACE ${o.trace_match_method}</span>`:''}${o.road_lane?` <span class="badge bRoad">${String(o.road_lane).toUpperCase()} d=${Number(o.road_d).toFixed(1)}</span>`:(o.road_lane_source==='c4_path_out_of_range'?'<span class="badge bRoad">PATH OUT</span>':'')}${o.canonical_valid?`<span class="badge bLink">${o.canonical_primary_domain||'CAN'} H${Number(o.canonical_handoff_count||0)}${o.canonical_reacquired?' R':''}</span>`:''}${o.kalman_valid?`<span class="badge bRoad">KF vy=${Number(o.kf_vy??0).toFixed(1)}${o.kf_d_dot==null?'':' ḋ='+Number(o.kf_d_dot).toFixed(1)}</span>`:''}${o.imm_valid?`<span class="badge bRoad">IMM ${o.imm_dominant_model||'?'} ${Math.round(100*Number(o.imm_prob_maneuver??0))}%M</span>`:''}${o.kf_cutin_candidate?`<span class="badge bL2">${o.kf_cutin_confirmed?'KF CUT':'KF CAND'} ${o.kf_ttlc_s==null?'':Number(o.kf_ttlc_s).toFixed(1)+'s'}${o.kf_cutin_confirmed?'':' p='+Number(o.kf_cutin_persistence_s??0).toFixed(1)}</span>`:''}${(!o.kf_cutin_candidate&&o.kf_low_speed_lateral_candidate)?`<span class="badge bRoad">KF LOW ${o.kf_ttlc_s==null?'':Number(o.kf_ttlc_s).toFixed(1)+'s'}</span>`:''}${o.preview_quality?`<span class="badge bLink">${o.preview_quality}</span>`:''}${o.scc_teacher_confirmed?'<span class="badge bScc">SCC</span>':''}${o.teacher_match?'<span class="badge bRear">LR/RR</span>':''}${o.front_link?'<span class="badge bLink">+FRONT</span>':''}${o.camera_confirmed?'<span class="badge bCam">CAM</span>':''}${(o.vehicle_duplicates_merged||0)>0?`<span class="badge bLink">MERGE×${Number(o.vehicle_member_count||1)}</span>`:''}${o.shadow_role==='L1'?'<span class="badge bL1">L1</span>':''}${o.shadow_role==='L2'?'<span class="badge bL2">L2</span>':''}${o.cutin_confirmed?'<span class="badge bL2">CUT-IN</span>':''}${mode==='raw'&&o.raw_address!=null?`<span class="badge" style="background:#153d5b;color:#cdeaff">0x${Number(o.raw_address).toString(16).toUpperCase()}</span>`:''}</span></div>`).join('')}
async function poll(){try{const r=await fetch('/state',{cache:'no-store'});state=await r.json();dirty=true;side();requestAnimationFrame(draw)}catch(e){}setTimeout(poll,100)}document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{mode=b.dataset.mode;document.querySelectorAll('[data-mode]').forEach(x=>x.classList.toggle('active',x===b));dirty=true;side();requestAnimationFrame(draw)});document.querySelectorAll('[data-range]').forEach(b=>b.onclick=()=>{rangeMode=b.dataset.range;document.querySelectorAll('[data-range]').forEach(x=>x.classList.toggle('active',x===b));buildBg();dirty=true;side();requestAnimationFrame(draw)});document.getElementById('hud').onclick=()=>{window.open('/hud','_blank')};document.getElementById('fs').onclick=async()=>{try{if(!document.fullscreenElement)await document.documentElement.requestFullscreen();else await document.exitFullscreen()}catch(e){}};poll();
</script></body></html>
'''

HUD_HTML=r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover"><title>G80 V38R2 Driver HUD</title><style>
html,body{margin:0;width:100%;height:100%;background:#05090c;color:#fff;font-family:Arial,"Noto Sans KR",sans-serif;overflow:hidden}
.wrap{height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center}
.card{position:relative;width:96vw;height:82vh;border:4px solid #34454f;border-radius:28px;background:#0b1216;display:flex;align-items:center;justify-content:center;overflow:hidden}
.center{z-index:3;min-width:40vw;max-width:52vw;text-align:center;padding:3vh 2vw;border-radius:22px;background:rgba(8,15,19,.80)}
.dir{font-size:5vw;font-weight:900}.state{font-size:9vw;font-weight:1000}.gaps{font-size:2.6vw;font-weight:800;margin-top:12px}.reason{font-size:1.9vw;margin-top:10px}.phase{font-size:1.7vw;margin-top:8px;color:#d0e0e8}.note{margin-top:1.2vh;font-size:1.35vw;color:#9eb1ba}
.sideArrow{position:absolute;top:50%;width:24vw;height:46vh;transform:translateY(-50%);opacity:.68;transition:all .12s;background:#455a64;filter:drop-shadow(0 8px 16px rgba(0,0,0,.55))}
.sideArrow.left{left:2vw;clip-path:polygon(100% 0,0 50%,100% 100%,78% 50%)}.sideArrow.right{right:2vw;clip-path:polygon(0 0,100% 50%,0 100%,22% 50%)}
.sideArrow.safe{background:#00c853}.sideArrow.check{background:#ffc400}.sideArrow.danger{background:#ff1744}.sideArrow.turn{background:#2979ff}.sideArrow.off{background:#455a64}
.sideArrow.active{opacity:1;transform:translateY(-50%) scale(1.08);filter:drop-shadow(0 0 28px rgba(255,255,255,.72))}.sideArrow.pulse{animation:pulse .72s ease-in-out infinite alternate}@keyframes pulse{from{opacity:.72}to{opacity:1}}
.sideLabel{position:absolute;bottom:3vh;font-size:1.5vw;font-weight:900;color:#d8e4e9;opacity:.82}.sideLabel.left{left:9vw}.sideLabel.right{right:9vw}
.safeText{color:#69f0ae}.checkText{color:#ffd740}.dangerText{color:#ff5252}.turnText{color:#64b5f6}.standby{color:#aebec6}
</style></head><body><div class="wrap"><div class="card"><div id="leftA" class="sideArrow left off"></div><div id="rightA" class="sideArrow right off"></div><div class="sideLabel left">LEFT</div><div class="sideLabel right">RIGHT</div><div class="center"><div id="dir" class="dir standby">PREVIEW</div><div id="st" class="state standby">READY</div><div id="gap" class="gaps">LEFT -- · RIGHT --</div><div id="why" class="reason">삼각형 색으로 양쪽 상태 미리보기</div><div id="ph" class="phase">V38R2 FG6 + SIGNAL PROBE</div></div></div><div class="note">초록 SAFE=차량 간격 + 차선 공간 확인 · 노랑 CHECK ROAD=도로/차선 불확실 · 회색 NO LANE=옆 차선 공간 없음 · 빨강 DANGER · 파랑 TURN. SHADOW 비교용이며 제어 신호가 아닙니다.</div></div>
<script>
const leftA=document.getElementById('leftA'),rightA=document.getElementById('rightA'),dir=document.getElementById('dir'),st=document.getElementById('st'),gap=document.getElementById('gap'),why=document.getElementById('why'),ph=document.getElementById('ph');
function cls(l){l=String(l||'');if(l.startsWith('SAFE'))return'safe';if(l.startsWith('DANGER'))return'danger';if(l.startsWith('CHECK'))return'check';if(l.startsWith('TURN'))return'turn';return'off'}
function textCls(l){const c=cls(l);return c==='safe'?'safeText':c==='danger'?'dangerText':c==='check'?'checkText':c==='turn'?'turnText':'standby'}
function reason(x){const m={'lane_change_commit_hold':'COMMIT HOLD','lane_change_rebase':'REBASE','intersection_turn_context':'TURN','commit_hold_hard_ttc_override':'TTC OVERRIDE','target_lane_absent':'NO TARGET LANE','target_lane_unconfirmed':'ROAD/LANE UNCERTAIN','front_gap<=5m':'FRONT GAP','rear_gap<=5m':'REAR GAP','boundary<=2m':'BOUNDARY','TTC<=3s':'TTC','stable_incoming_near':'INCOMING','front_gap<=12m':'FRONT GAP','rear_gap<=12m':'REAR GAP','boundary<=5m':'BOUNDARY','TTC<=5s':'TTC'};return m[x]||String(x||'').replaceAll('_',' ').toUpperCase()}
async function tick(){try{const s=await(await fetch('/state',{cache:'no-store'})).json(),fg=s.future_gap||{},d=fg.driver_intent||{},ll=fg.left?.decision?.label||'--',rr=fg.right?.decision?.label||'--';let lc=cls(ll),rc=cls(rr);if(d.active){if(d.maneuver_context==='TURN'){if(d.side==='left')lc='turn';if(d.side==='right')rc='turn'}else{const ac=cls(d.label||'CHECK ?');if(d.side==='left')lc=ac;if(d.side==='right')rc=ac}}leftA.className=`sideArrow left ${lc}${d.active&&d.side==='left'?' active':''}${d.active&&d.side==='left'&&d.phase==='COMMIT_HOLD'?' pulse':''}`;rightA.className=`sideArrow right ${rc}${d.active&&d.side==='right'?' active':''}${d.active&&d.side==='right'&&d.phase==='COMMIT_HOLD'?' pulse':''}`;if(!d.active){dir.textContent='PREVIEW';dir.className='dir standby';st.textContent='READY';st.className='state standby';gap.textContent=`LEFT ${ll} · RIGHT ${rr}`;why.textContent='양쪽 삼각형 색을 먼저 확인';ph.textContent='V38 DEC3 · BLINKER OFF';}else if(d.maneuver_context==='TURN'||d.state==='TURN'){dir.textContent=d.side==='left'?'← LEFT':'RIGHT →';dir.className='dir turnText';st.textContent='TURN';st.className='state turnText';gap.textContent=`${(Number(fg.ego?.v_ego_mps||0)*3.6).toFixed(1)} km/h · steering ${Number(d.steering_angle_deg||0).toFixed(0)}°`;why.textContent='교차로/저속 회전으로 판단 · lane-change 판단 숨김';ph.textContent='TURN CONTEXT';}else{const l=d.label||'CHECK ?';dir.textContent=d.side==='left'?'← LEFT':'RIGHT →';dir.className='dir '+textCls(l);st.textContent=l;st.className='state '+textCls(l);const f=d.front_clearance_m==null?'--':Number(d.front_clearance_m).toFixed(1),r=d.rear_clearance_m==null?'--':Number(d.rear_clearance_m).toFixed(1);gap.textContent=`FRONT ${f} m · REAR ${r} m`;why.textContent=(d.reasons||[]).slice(0,3).map(reason).join(' · ')||'NO ACTIVE REASON';ph.textContent=(d.phase||'PRECHECK')+(d.hold_remaining_s==null?'':` · HOLD ${Number(d.hold_remaining_s).toFixed(1)}s`);}}catch(e){}setTimeout(tick,100)}tick();
</script></body></html>'''
class H(BaseHTTPRequestHandler):
  def log_message(self,*a): pass
  def do_GET(self):
    path=self.path.split('?')[0]
    if path=='/state':
      mark_browser_active()
      with state_lock: b=latest_state_json
      self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    elif path=='/hud':
      mark_browser_active();b=HUD_HTML.encode()
      self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    else:
      b=HTML.encode();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)

def http_thread(): ThreadingHTTPServer(('0.0.0.0',HTTP_PORT),H).serve_forever()
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
  'key','source','x','y','vx','sector','front_sector','vehicle_key','canonical_key','canonical_valid',
  'canonical_primary_domain','canonical_domains','canonical_domain_history','canonical_handoff_count','canonical_reacquired','canonical_gap_ms','source_mask','source_age_ms','display_state','trace_local_key','trace_match_method','trace_unmatched','trace_canonical_domains','corner_debug_role','road_d','road_lane','road_lane_source',
  'corner_link_id','vehicle_cluster_sources','kalman_valid','kf_dormant_preserved','kf_x','kf_y','kf_vy','kf_d_dot','kf_cutin_candidate',
  'kf_cutin_confirmed','kf_ttlc_s','kf_cutin_persistence_s','kf_low_speed_lateral_candidate','kalman_trajectory',
  'imm_valid','imm_dominant_model','imm_prob_maneuver','imm_eval_age_ms','scc_teacher_confirmed','teacher_match',
  'front_link','camera_confirmed','vehicle_duplicates_merged','vehicle_member_count','shadow_role','cutin_confirmed',
  'preview_quality','raw_address'
)
def _ui_obj(o):
  return {k:o.get(k) for k in _UI_OBJ_KEYS if k in o and o.get(k) is not None}

_UI_TOP_KEYS = (
  'version','mono_ns','runtime_versions','runtime_mismatch','coordinate_frame','road_model','rear_teacher_match_stats',
  'canonical_tracker_stats','view_consistency_stats','kalman_motion_stats','imm_motion_stats','future_gap','traffic_signal_probe','ego_state','camera_fusion_stats',
  'shadow_leads','corner_front_associations','zones','teacher_rear','scc_teacher','scc_front_match','corner_fusion_stats',
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
  tracks=TrackStore(ttl_s=.75,continuity_s=.30);corner_fuser=CornerFusionTracker();corner_vehicle_tracker=VehicleFootprintTracker('VC');scc_matcher=SccFrontTeacherMatcher();camera_fuser=CameraRadarFusion();canonical_tracker=Canonical360Tracker('V',ttl_s=1.5);shadow_verifier=ShadowLeadVerifier();shadow_logger=ShadowLogger();front_preview=StandardFrontPreview();motion_tracker=KalmanMotionTracker('KF');imm_tracker=ImmMotionTracker();future_gap=FutureGapEvaluator();signal_probe=TrafficSignalProbe();camera_leads=[];model_path=[];road_model={};model_path_recv_ns=0;v_ego=0.0;a_ego=0.0;steering_angle_deg=0.0;left_blinker=False;right_blinker=False;v_ego_recv_ns=0;production=None;rear_teacher=[];rear_teacher_by_bus={};scc_teacher=None;scc_teacher_by_bus={};SCC_BUS=int(os.getenv('G80_SCC_BUS',str(DEFAULT_SCC_BUS)))
  udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
  diag={'corner_A_frames':0,'corner_B_frames':0,'corner_decoded_total':0,'corner_rear_total':0,'corner_front_total':0,'corner_A_by_bus':{},'corner_B_by_bus':{},'scc_frames_by_bus':{},'scc_teacher_updates':0,'model_frames':0,'camera_leads_latest':0,'model_transport_lag_ms':None,'last_transport_lag_ms':None}
  next_pub=time.monotonic();next_debug_write=0.0;next_ui_update=0.0;last_pub_ns=0;last_processing_ms=0.0;last_udp_ms=0.0;last_ui_ms=0.0;last_ui_kb=0.0;last_stage_ms={}
  while True:
    processed=0
    for _ in range(2000):
      msg=messaging.recv_one_or_none(can_sock)
      if msg is None: break
      recv_ns=time.monotonic_ns();can_log_ns=int(msg.logMonoTime);diag['last_transport_lag_ms']=round((recv_ns-can_log_ns)/1e6,3)
      for f in msg.can:
        bus,addr,dat=int(f.src),int(f.address),bytes(f.dat)
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

    now=time.monotonic()
    if now>=next_pub:
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
      st=time.perf_counter()
      raw=tracks.snapshot(now_ns);filt=filtered_objects(raw,rear_teacher);cf=corner_fuser.update(filt,now_ns);fronts=build_front_objects(filt);combined=associate_corner_front(cf['corner_fused_objects'],fronts);corners=combined['corner_objects'];fronts=combined['front_objects'];radar_all_fused=combined['all_fused_objects'];assocs=combined['associations'];camf=camera_fuser.update(radar_all_fused,camera_leads,now_ns,fronts);sensor_fused_clusters=camf['sensor_fused_objects'];front_sensor=camf['front_sensor_objects'];corner_vehicle,corner_vehicle_stats=corner_vehicle_tracker.update(corners,now_ns);stdp=front_preview.update(raw,fronts,camera_leads,scc_teacher,now_ns)
      stage_ms['core_fusion']=(time.perf_counter()-st)*1000.0
      st=time.perf_counter();road_view=road_model_with_age(road_model,now_ns)
      # V34: road projection is authoritative only for the Canonical sensor-fused
      # set. Diagnostic RAW/FILTERED/CORNER/FRONT tabs keep ego-frame x/y without
      # repeated Frenet projection; this removes a dense-traffic UI cost.
      raw_view=raw if (ui_active or debug_due) else []
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
      st=time.perf_counter();future_gap_state=future_gap.update(sensor_fused_view,v_ego,a_ego,left_blinker,right_blinker,now_ns,steering_angle_deg=steering_angle_deg,road_curve_direction=road_view.get('curve_direction'),road_model=road_view);stage_ms['future_gap']=(time.perf_counter()-st)*1000.0
      has_prod_lead=bool(production and (production.get('leadOne') or {}).get('status'))
      traffic_signal_state=signal_probe.snapshot(now_ns,has_lead=has_prod_lead)
      st=time.perf_counter();shadow=shadow_verifier.update(sensor_fused_view,model_path,v_ego,now_ns,production,model_path_recv_ns,v_ego_recv_ns,scc_teacher);stage_ms['shadow']=(time.perf_counter()-st)*1000.0
      # V38R2: Canonical360 stays full coverage; selective KF4/IMM3 feed FG6 at the 10 Hz publication loop.
      corner_vehicle_view=_trace_diag_to_canonical(corner_vehicle_road,sensor_fused_view,now_ns,'corner');corner_kalman_stats={'disabled_in_v32':True}
      corner_candidate_view=_trace_diag_to_canonical(corner_candidates_raw,sensor_fused_view,now_ns,'corner')
      corner_debug_view=corner_vehicle_view+corner_candidate_view
      front_sensor_view=_trace_diag_to_canonical(front_sensor_road,sensor_fused_view,now_ns,'front');front_kalman_stats={'disabled_in_v32':True}
      fronts_view=_trace_diag_to_canonical(fronts_view,sensor_fused_view,now_ns,'front')
      std_points_view=_trace_diag_to_canonical(std_points_view,sensor_fused_view,now_ns,'stdpreview')
      filt_view=_trace_diag_to_canonical(filt_view,sensor_fused_view,now_ns,'filtered') if (ui_active or debug_due) else []
      view_consistency_stats=_view_consistency(sensor_fused_view,front_sensor_view,corner_vehicle_view,corner_candidate_view)
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
      runtime_versions={'build':BUILD_VERSION,'tag':BUILD_TAG,'logger':LOGGER_SERVICE_VERSION,'kalman_api':KALMAN_API_VERSION,'imm_api':IMM_API_VERSION,'future_gap_api':FUTURE_GAP_API_VERSION,'android_protocol':PROTOCOL_VERSION}
      runtime_mismatch=(BUILD_VERSION!=38 or LOGGER_SERVICE_VERSION!=BUILD_VERSION or KALMAN_API_VERSION!=4 or IMM_API_VERSION!=3 or FUTURE_GAP_API_VERSION!=6 or BUILD_FUTURE_GAP_API_VERSION!=6 or PROTOCOL_VERSION!=21)
      core={'version':BUILD_VERSION,'mono_ns':now_ns,'runtime_versions':runtime_versions,'runtime_mismatch':runtime_mismatch,'objects':sensor_fused_view,
            'sensor_fused_objects':sensor_fused_view,'all_fused_objects':sensor_fused_view,'canonical360_objects':sensor_fused_view,
            'radar_fused_objects':radar_all_view,
            'corner_fused_objects':corner_vehicle_view,'corner_candidate_objects':corner_candidate_view,'corner_debug_objects':corner_debug_view,'corner_radar_objects':corners_view,'front_objects':fronts_view,'front_sensor_objects':front_sensor_view,
            'standard_front_preview':std_points_view,'standard_front_preview_stats':stdp['stats'],
            'camera_leads':camera_leads_view,'camera_fusion_matches':camf['camera_matches'],
            'road_model':road_view,'coordinate_frame':coordinate_frame,'rear_teacher_match_stats':rear_teacher_match_stats,
            'canonical_tracker_stats':canonical_stats,'view_consistency_stats':view_consistency_stats,'kalman_motion_stats':dict(kalman_stats,canonical_identity_input=True),'imm_motion_stats':imm_stats,'future_gap':future_gap_state,'traffic_signal_probe':traffic_signal_state,'ego_state':{'vEgo':v_ego,'aEgo':a_ego,'steeringAngleDeg':steering_angle_deg,'leftBlinker':left_blinker,'rightBlinker':right_blinker},'corner_kalman_motion_stats':corner_kalman_stats,'front_kalman_motion_stats':front_kalman_stats,
            'camera_fusion_stats':dict(camf['stats'],corner_vehicle_objects_before=corner_vehicle_stats['vehicle_objects_before'],corner_vehicle_objects_after=corner_vehicle_stats['vehicle_objects_after'],corner_vehicle_duplicates_merged=corner_vehicle_stats['vehicle_duplicates_merged']),'shadow_leads':shadow,
            'corner_front_associations':assocs,'zones':zones,
            'teacher_rear':rear_teacher,'scc_teacher':scc_teacher or {},
            'scc_teacher_by_bus':{str(k):v for k,v in scc_teacher_by_bus.items()},
            'scc_front_match':scc_match_status,'corner_fusion_stats':cf['stats'],
            'shadow_logger':shadow_logger.status()}
      interval_ms=None if last_pub_ns==0 else (now_ns-last_pub_ns)/1e6
      core['performance_stats']={'processing_ms':round(last_processing_ms,2),'publish_interval_ms':None if interval_ms is None else round(interval_ms,2),'target_hz':PUBLISH_HZ,'local_kf_replicas_disabled':True,'imm_target_hz':imm_stats.get('target_hz',3.0),'stage_ms':{k:round(v,2) for k,v in stage_ms.items()},'udp_json_ms':round(last_udp_ms,2),'ui_json_ms':round(last_ui_ms,2),'logger_event_policy':'lead/maneuver transitions only; 2Hz periodic'}

      # Persistent shadow evaluation log under /data/radar (outside the git tree).
      # 2 Hz periodic by default, plus immediate records on important state changes.
      shadow_logger.maybe_write(core,model_path,v_ego,model_path_recv_ns,v_ego_recv_ns,diag,now_ns)
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
          'diagnostics':dict(diag,ui_sleeping=not ui_active),
          'browser_active':ui_active,
          'notes':{
            'ui':'v38r2-source-trace-road-gate-signal-probe-corner-debug-goldenv2',
            'display_scale':'SHORT/LONG 1:1 + WIDE + DRIVE perspective; source-traceable Canonical C4-road overlay',
            'browser_policy':'8Hz compact cached JSON; no duplicate canonical arrays; diagnostic tabs stay ego-frame',
            'udp':'Android final sensor-fusion packet on 28991',
            'control':'disabled',
            'integration':'NOT connected to RadarInterface/radarTracks/radard',
            'future_front':'plain-dict RadarPoint preview only',
            'future_gap':'FG6 DEC3 road/lane SAFE gate + turn/lane-change context + commit hold/rebase + dual-arrow driver HUD',
            'identity':'Canonical360Tracker is the only global Vxxxx authority; diagnostic views are traced back to Vxxxx when possible',
            'motion_prediction':'Selective KF4 max12 + dormant preserve; IMM3 max8@3Hz; FG6 consumes cached IMM/KF trajectories',
            'runtime_versions':runtime_versions,'runtime_mismatch':runtime_mismatch,
            'shadow_control':'NEVER publishes radarTracks/radarState / NEVER CAN TX',
            'debug_state_hz':DEBUG_STATE_HZ,'ui_state_hz':UI_STATE_HZ,'shadow_log_dir':'/data/radar'
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
                 'shadow_logger':core.get('shadow_logger',{}),'diagnostics':dict(diag,ui_sleeping=True)}
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
      # Fixed-phase scheduler: processing time no longer gets added to the 100 ms period.
      period=1.0/max(PUBLISH_HZ,1.0);next_pub+=period;end_now=time.monotonic()
      if next_pub<end_now:
        next_pub += (int((end_now-next_pub)/period)+1)*period
    if processed==0: time.sleep(.002)
if __name__=='__main__': main()
