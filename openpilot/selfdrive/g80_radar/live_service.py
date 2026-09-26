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
from openpilot.selfdrive.g80_radar.android_packet import build_render_packet
from openpilot.selfdrive.g80_radar.camera_fusion import decode_model_leads,CameraRadarFusion,VehicleFootprintTracker
from openpilot.selfdrive.g80_radar.shadow_leads import ShadowLeadVerifier,snapshot_production_radar_state
from openpilot.selfdrive.g80_radar.shadow_logger import ShadowLogger
from openpilot.selfdrive.g80_radar.front_standard_preview import StandardFrontPreview
from openpilot.selfdrive.g80_radar.road_geometry import extract_road_model,road_model_with_age,path_as_tuples,annotate_objects

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

def mark_browser_active():
  global browser_last_poll_ns
  with browser_lock:
    browser_last_poll_ns=time.monotonic_ns()

def browser_is_active(now_ns=None):
  if now_ns is None: now_ns=time.monotonic_ns()
  with browser_lock:
    last=browser_last_poll_ns
  return last>0 and (now_ns-last)<=BROWSER_ACTIVE_NS

latest_state={'version':21,'objects':[],'corner_fused_objects':[],'front_objects':[],'standard_front_preview':[],'all_fused_objects':[],'filtered_objects':[],'raw_objects':[]}

HTML=r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover"><title>G80 5-Radar V21 Monitor</title><style>
:root{--bg:#061018;--panel:#091720;--line:#24404f;--text:#eaf4f8;--muted:#8fa8b5;--cyan:#16d4e3;--green:#57d96a;--gold:#ffd34e;--purple:#cf4eff;--orange:#ff913c;--blue:#168dff}*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%;overflow:hidden;background:radial-gradient(circle at 50% 0,#0d2432 0,#061018 46%,#03090e 100%);color:var(--text);font-family:Arial,"Noto Sans KR",sans-serif}body{display:flex;flex-direction:column}.top{height:58px;display:flex;align-items:center;gap:7px;padding:7px 10px;background:rgba(4,13,19,.96);border-bottom:1px solid #2d4b5c}.tabs,.range{display:flex;gap:5px}.tabs button,.range button,#fs{border:1px solid #35566a;border-radius:8px;background:#0a1822;color:#b9cad4;padding:9px 13px;font-size:12px;font-weight:700}.tabs button.active,.range button.active{color:white;background:linear-gradient(#118eff,#0865c2);border-color:#63bcff;box-shadow:inset 0 0 0 1px #6bc2ff}.spacer{flex:1}.brand{text-align:right;line-height:1.12;margin-right:8px}.brand b{font-size:16px}.brand em{font-style:normal;color:var(--cyan);font-weight:700}.live{font-size:11px;color:#68ff83;border:1px solid #28513a;padding:5px 8px;border-radius:8px}.main{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) 342px;gap:8px;padding:8px}.scene,.side{border:1px solid #29495a;border-radius:14px;background:linear-gradient(180deg,rgba(8,23,32,.96),rgba(4,13,19,.96));overflow:hidden}.scene{position:relative}.scene canvas{width:100%;height:100%;display:block}.side{padding:10px;display:flex;flex-direction:column;gap:8px}.card{border:1px solid #263f4e;border-radius:10px;background:#08151d;padding:9px}.card h3{margin:0 0 7px;font-size:14px}.metric{display:flex;justify-content:space-between;font-size:12px;line-height:1.55;gap:12px}.metric span:first-child{color:var(--muted)}.metric b{text-align:right}.teacher{color:var(--gold)}.rear{color:var(--purple)}.bad{color:#ff6b6b}.dim{color:#8fa8b5}.fusion{color:var(--cyan)}.front{color:var(--green)}.zones{display:grid;grid-template-columns:repeat(5,1fr);gap:4px}.zone{text-align:center;border:1px solid #334b58;background:#0b1c25;border-radius:7px;padding:6px 2px;font-size:10px}.zone.on{background:#243518;border-color:#587c37;color:#a5ff75}.objList{font-family:ui-monospace,Consolas,monospace;font-size:10.5px;overflow:auto;min-height:0;flex:1}.obj{display:grid;grid-template-columns:54px 38px 1fr;gap:4px;padding:5px 2px;border-bottom:1px solid #152a35}.obj .id{font-weight:700}.badge{display:inline-block;border-radius:4px;padding:1px 4px;font-size:8.5px;margin-left:3px}.bScc{background:#7a5a00;color:#fff1a8}.bRear{background:#662276;color:#ffd6ff}.bLink{background:#174f32;color:#aaffc3}.bCam{background:#164d78;color:#d3efff}.bL1{background:#695400;color:#ffe78a}.bL2{background:#6b2c0d;color:#ffbd8f}.bRoad{background:#19485b;color:#b7f5ff}.footer{font-size:9.5px;color:#78909d;line-height:1.35}@media(max-width:900px){.main{grid-template-columns:1fr;grid-template-rows:minmax(0,1fr) 220px}.side{display:grid;grid-template-columns:1fr 1fr 1fr;overflow:auto}.objList{grid-column:1/4}.brand{display:none}.tabs button,.range button{padding:7px 7px;font-size:10px}.top{height:52px;padding:5px}}
</style></head><body><div class="top"><div class="tabs"><button data-mode="corner" class="active">CORNER FUSED</button><button data-mode="front">FRONT</button><button data-mode="stdpreview">STD PREVIEW</button><button data-mode="all">ALL FUSED</button><button data-mode="filtered">FILTERED</button><button data-mode="raw">RAW</button><button data-mode="shadow">SHADOW L1/L2</button></div><div class="range"><button data-range="short" class="active">SHORT 1:1</button><button data-range="long">LONG 1:1</button><button data-range="wide">WIDE</button><button data-range="drive">DRIVE 3-LANE</button></div><div class="spacer"></div><div class="brand"><b>G80 5-Radar (v21)</b><br><em>MONITOR ONLY · Perspective Road</em></div><div class="live">● LIVE 10 Hz</div><button id="fs">전체</button></div><div class="main"><div class="scene"><canvas id="radar"></canvas></div><div class="side"><div class="card"><h3>상태</h3><div id="summary"></div></div><div class="card"><h3>차선 점유</h3><div id="zones" class="zones"></div></div><div class="card"><h3>Teacher</h3><div id="teachers"></div></div><div id="objects" class="card objList"></div><div class="footer">v21 MONITOR ONLY: C4 curved-road + perspective display + corner-sector colors. No radarTracks/radarState publish, no CAN TX.</div></div></div><script>
const cvs=document.getElementById('radar'),ctx=cvs.getContext('2d',{alpha:false});const scene=cvs.parentElement,summary=document.getElementById('summary'),zonesEl=document.getElementById('zones'),teachers=document.getElementById('teachers'),objectsEl=document.getElementById('objects');let mode='corner',rangeMode='short',state=null,dirty=true,lastSideUpdate=0;let cssW=1,cssH=1,dpr=1;const bg=document.createElement('canvas'),bgc=bg.getContext('2d',{alpha:false});const C={grid:'#1b3340',lane:'#8099a6',edge:'#415661',text:'#eaf4f8',muted:'#829ba8',cyan:'#16d4e3',green:'#57d96a',gold:'#ffd34e',purple:'#cf4eff',orange:'#ff913c',gray:'#9cb1bc',camera:'#76c7ff',FL:'#00e5ff',FR:'#ff5fc8',RL:'#8cff52',RR:'#ff9f43',FC:'#5ad0ff',RC:'#c7a7ff'};
function q(){if(rangeMode==='short')return{rear:-15,front:35,step:5,stretch:false};if(rangeMode==='long')return{rear:-50,front:100,step:10,stretch:false};if(rangeMode==='wide')return{rear:-30,front:70,step:10,stretch:true,lat:12.6,wide:true};return{rear:-12,front:40,step:5,stretch:true,lat:7.2,drive:true,perspective:true}}function mppY(){const a=q();return cssH/(a.front-a.rear)}function mppX(){const a=q();return a.stretch?cssW/(2*a.lat):mppY()}function persp(x){const a=q();if(!a.perspective)return 1;const f=Math.max(0,Number(x));return Math.max(.50,1/(1+f/55))}function X(y,x=0){return cssW/2-y*mppX()*persp(x)}function Y(x){return(q().front-x)*mppY()}function visible(x,y){const a=q(),half=a.stretch?a.lat:cssW/(2*mppX());return x>=a.rear&&x<=a.front&&y>=-half/persp(x)&&y<=half/persp(x)}function rounded(c,x,y,w,h,r,fill,stroke,lw=1){c.beginPath();c.roundRect(x,y,w,h,r);if(fill){c.fillStyle=fill;c.fill()}if(stroke){c.strokeStyle=stroke;c.lineWidth=lw;c.stroke()}}
function car(c,xm,ym,color,label,highlight=false){const a=q(),cx=X(ym,xm),cy=Y(xm);let W,H;if(a.drive){const ps=persp(xm),ego=Math.abs(xm)<.3&&Math.abs(ym)<.3;W=(ego?50:44)*ps;H=(ego?82:72)*ps;W=Math.max(26,W);H=Math.max(42,H)}else if(a.wide){W=26;H=50}else{W=Math.max(14,1.8*mppX());H=Math.max(28,4.5*mppY())}rounded(c,cx-W/2,cy-H/2,W,H,Math.min(8,W*.28),color+'44',color,1.6);rounded(c,cx-W*.27,cy-H*.28,W*.54,H*.56,Math.min(4,W*.16),'#263c49','#76909e',.8);c.fillStyle=color;c.fillRect(cx-W*.40,cy-H*.44,W*.80,Math.max(2,H*.06));if(highlight){c.strokeStyle=C.gold;c.lineWidth=3;c.beginPath();c.arc(cx,cy,Math.max(W,H)*.62,0,Math.PI*2);c.stroke();c.fillStyle=C.gold;c.font='bold 14px Arial';c.fillText('★',cx+W*.55,cy-H*.48)}c.fillStyle=C.text;c.font='bold 10px Arial';c.fillText(label,cx+W*.62,cy-3);return[cx,cy,W,H]}
function buildBg(){bg.width=cvs.width;bg.height=cvs.height;bgc.setTransform(dpr,0,0,dpr,0,0);const g=bgc.createLinearGradient(0,0,0,cssH);g.addColorStop(0,'#0a1d28');g.addColorStop(.55,'#07141c');g.addColorStop(1,'#030a0f');bgc.fillStyle=g;bgc.fillRect(0,0,cssW,cssH);const a=q();bgc.font='11px Arial';bgc.textAlign='left';for(let x=Math.ceil(a.rear/a.step)*a.step;x<=a.front;x+=a.step){const yy=Y(x);bgc.strokeStyle=x===0?'#77909c':C.grid;bgc.lineWidth=x===0?1.5:1;bgc.beginPath();bgc.moveTo(0,yy);bgc.lineTo(cssW,yy);bgc.stroke();bgc.fillStyle=C.muted;bgc.fillText((x>0?'+':'')+x+'m',6,yy-4)}if(a.drive){bgc.fillStyle='rgba(255,255,255,.80)';bgc.font='bold 11px Arial';bgc.fillText('DRIVE · 3 LANES · C4 ROAD MODEL',10,17)}else if(a.wide){bgc.fillStyle='rgba(255,255,255,.72)';bgc.font='bold 10px Arial';bgc.fillText('WIDE · C4 ROAD MODEL',10,16)}else{const bar=5*mppX();bgc.strokeStyle='#d8e6ec';bgc.lineWidth=2;bgc.beginPath();bgc.moveTo(cssW-20-bar,cssH-20);bgc.lineTo(cssW-20,cssH-20);bgc.stroke();bgc.fillStyle=C.muted;bgc.fillText('5 m',cssW-20-bar/2-10,cssH-25)}}
function resize(){const r=scene.getBoundingClientRect(),nd=Math.min(window.devicePixelRatio||1,2);const nw=Math.max(2,Math.floor(r.width*nd)),nh=Math.max(2,Math.floor(r.height*nd));if(nw!==cvs.width||nh!==cvs.height){dpr=nd;cssW=r.width;cssH=r.height;cvs.width=nw;cvs.height=nh;ctx.setTransform(dpr,0,0,dpr,0,0);buildBg();dirty=true}}window.addEventListener('resize',resize);new ResizeObserver(resize).observe(scene);resize();
function poly(points,color,width=1.5,dash=[]){if(!points||points.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();let started=false;for(const p of points){const x=Number(p.x),y=Number(p.y);if(!Number.isFinite(x)||!Number.isFinite(y))continue;const sx=X(y,x),sy=Y(x);if(!started){ctx.moveTo(sx,sy);started=true}else ctx.lineTo(sx,sy)}if(started)ctx.stroke();ctx.setLineDash([])}
function roadYAtX(x){const rm=state?.road_model||{},p=rm.path||[];if(!rm.fresh||p.length<2)return null;const xmax=Number(rm.path_x_max_m??p[p.length-1].x),margin=Number(rm.path_projection_margin_m??4);if(x>xmax+margin)return null;let prev=p[0];for(let i=1;i<p.length;i++){const cur=p[i];if(Number(cur.x)>=x){const dx=Number(cur.x)-Number(prev.x);if(Math.abs(dx)<1e-6)return Number(prev.y);const t=(x-Number(prev.x))/dx;return Number(prev.y)+(Number(cur.y)-Number(prev.y))*t}prev=cur}return null}
function offsetPath(offset){const rm=state?.road_model||{},p=rm.path||[];if(p.length<2)return[];const out=[];for(let i=0;i<p.length;i++){const a=p[Math.max(0,i-1)],b=p[Math.min(p.length-1,i+1)],dx=Number(b.x)-Number(a.x),dy=Number(b.y)-Number(a.y),n=Math.hypot(dx,dy)||1;const nx=-dy/n,ny=dx/n;out.push({x:Number(p[i].x)+nx*offset,y:Number(p[i].y)+ny*offset})}return out}
function straightFallback(x0,x1,offset){return[{x:x0,y:offset},{x:x1,y:offset}]}
function drawRoad(){const a=q(),rm=state?.road_model||{},fresh=!!rm.fresh&&(rm.path||[]).length>1;const laneW=3.6,outer=laneW*1.5;const offsets=a.drive?[-outer,-laneW/2,laneW/2,outer]:[-9,-5.4,-1.8,1.8,5.4,9];if(fresh){let actualLines=(rm.lane_lines||[]).filter(ln=>Number(ln.prob||0)>=.35&&Array.isArray(ln.points)&&ln.points.length>1);let fillLeft=offsetPath(outer),fillRight=offsetPath(-outer);if(a.drive&&actualLines.length>=4){const meanY=ln=>{const pts=ln.points||[];if(!pts.length)return 0;const n=Math.min(8,pts.length);let z=0;for(let i=0;i<n;i++)z+=Number(pts[i].y)||0;return z/n};const sorted=actualLines.slice().sort((u,v)=>meanY(v)-meanY(u));fillLeft=sorted[0].points||fillLeft;fillRight=sorted[sorted.length-1].points||fillRight}if(fillLeft.length>1&&fillRight.length>1){ctx.fillStyle=a.drive?'rgba(22,212,227,.050)':'rgba(22,212,227,.036)';ctx.beginPath();for(let i=0;i<fillLeft.length;i++){const p=fillLeft[i];if(i===0)ctx.moveTo(X(p.y,p.x),Y(p.x));else ctx.lineTo(X(p.y,p.x),Y(p.x))}for(let i=fillRight.length-1;i>=0;i--){const p=fillRight[i];ctx.lineTo(X(p.y,p.x),Y(p.x))}ctx.closePath();ctx.fill()}if(!a.drive||actualLines.length<3){for(const off of offsets)poly(offsetPath(off),'rgba(92,119,133,.30)',1,[7,10])}for(const off of offsets)poly(straightFallback(a.rear,0,off),'rgba(92,119,133,.30)',1,[7,10]);for(const e of (rm.road_edges||[]))poly(e.points||[],C.edge,1,[3,8]);let actual=0;for(const ln of (rm.lane_lines||[])){const p=Number(ln.prob||0);if(p<.20)continue;actual++;ctx.globalAlpha=Math.max(.28,Math.min(1,.30+.70*p));poly(ln.points||[],C.lane,p>.6?2.2:1.35,[10,8]);ctx.globalAlpha=1}poly(rm.path||[],C.cyan,2.0,[]);if(a.drive){const labelX=Math.min(10,a.front*.30),base=roadYAtX(labelX)??0;ctx.textAlign='center';ctx.font='bold 10px Arial';ctx.fillStyle=C.muted;for(const p of [['L1',laneW],['EGO',0],['R1',-laneW]])ctx.fillText(p[0],X(base+p[1],labelX),Y(labelX)-4);ctx.textAlign='left'}ctx.fillStyle='rgba(183,245,255,.82)';ctx.font='bold 10px Arial';ctx.fillText(`C4 ${rm.curve_direction||'ROAD'} · lane ${actual}/${rm.confident_lane_lines??0} · horizon ${rm.path_x_max_m==null?'--':Number(rm.path_x_max_m).toFixed(0)+'m'} · ${rm.age_ms==null?'--':Number(rm.age_ms).toFixed(0)+'ms'}`,10,34)}else{for(const off of offsets)poly(straightFallback(a.rear,a.front,off),C.lane,1,[9,9]);ctx.fillStyle='rgba(255,107,107,.9)';ctx.font='bold 10px Arial';ctx.fillText('C4 ROAD MODEL STALE/UNAVAILABLE · straight fallback',10,34)}}
function arr(){if(!state)return[];if(mode==='corner')return state.corner_fused_objects||[];if(mode==='front')return state.front_sensor_objects||state.front_objects||[];if(mode==='stdpreview')return state.standard_front_preview||[];if(mode==='all')return state.sensor_fused_objects||state.all_fused_objects||[];if(mode==='filtered')return state.filtered_objects||[];if(mode==='shadow')return state.shadow_leads?.candidates||[];return state.raw_objects||[]}function cornerSector(o){const src=String(o.source||''),sources=o.vehicle_cluster_sources||[];const hasCorner=src.startsWith('corner')||o.corner_link_id!=null||sources.some(v=>String(v).startsWith('corner'));if(!hasCorner)return null;const s=o.sector;if(['FL','FR','RL','RR','FC','RC'].includes(s))return s;const x=Number(o.x),y=Number(o.y);if(x>.5&&y>1.2)return'FL';if(x>.5&&y<-1.2)return'FR';if(x<-.5&&y>1.2)return'RL';if(x<-.5&&y<-1.2)return'RR';if(x>.5)return'FC';if(x<-.5)return'RC';return null}
function srcColor(o){if(o.shadow_role==='L1')return C.gold;if(o.shadow_role==='L2')return C.orange;const sec=cornerSector(o);if(sec&&C[sec])return C[sec];if(o.source==='c4_camera')return C.camera;if(o.source==='front_track'||o.source==='fr_cmr_reference')return C.green;if(o.source==='future_standard_front_preview')return C.gold;if((o.source||'').includes('front_group1'))return C.orange;return C.gray}
function drawTeacherMarkers(){if(!state)return;const rear=state.teacher_rear||[];for(const t of rear){const d=Number(t.distance_candidate_m);if(!Number.isFinite(d)||d<=0)continue;const y=t.sector==='LR'?3.6:-3.6,x=-d;if(!visible(x,y))continue;const xx=X(y,x),yy=Y(x),ok=!!t.teacher_usable;ctx.strokeStyle=ok?C.purple:'#76577e';ctx.lineWidth=ok?2.5:1.4;ctx.beginPath();ctx.moveTo(xx-7,yy);ctx.lineTo(xx+7,yy);ctx.moveTo(xx,yy-7);ctx.lineTo(xx,yy+7);ctx.stroke();ctx.fillStyle=ok?C.purple:'#8d7195';ctx.font='bold 9px Arial';ctx.fillText(`${t.sector} T ${d.toFixed(1)}m S${t.status_raw??'?'}`,xx+9,yy-8)}const s=state.scc_teacher||{};if(s.distance_m!=null){const x=Number(s.distance_m),py=roadYAtX(x),y=Number.isFinite(py)?py:0;if(Number.isFinite(x)&&visible(x,y)){const xx=X(y,x),yy=Y(x),ok=!!s.teacher_usable;ctx.strokeStyle=ok?C.gold:C.orange;ctx.lineWidth=ok?3:1.5;ctx.beginPath();ctx.moveTo(xx-9,yy);ctx.lineTo(xx+9,yy);ctx.moveTo(xx,yy-9);ctx.lineTo(xx,yy+9);ctx.stroke();ctx.fillStyle=ok?C.gold:C.orange;ctx.font='bold 10px Arial';ctx.fillText(`SCC T B${s.bus??'?'} ${x.toFixed(1)}m${Number.isFinite(py)?' · PATH':''}`,xx+12,yy-9)}}}
function draw(){if(!dirty)return;dirty=false;ctx.drawImage(bg,0,0,bg.width,bg.height,0,0,cssW,cssH);drawRoad();car(ctx,0,0,'#f3f7f8','G80');for(const o of arr()){const x=Number(o.x),y=Number(o.y);if(!Number.isFinite(x)||!Number.isFinite(y)||!visible(x,y))continue;const col=srcColor(o),lane=o.road_lane?` ${String(o.road_lane).toUpperCase()}`:(o.road_lane_source==='c4_path_out_of_range'?' PATH?':''),id=(o.vehicle_key||o.key||'?')+(o.sector||o.front_sector?' '+(o.sector||o.front_sector):'')+(rangeMode==='drive'?lane:'');const p=car(ctx,x,y,col,id,!!o.scc_teacher_confirmed);if(o.teacher_match){ctx.strokeStyle=C.purple;ctx.lineWidth=3;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*.70,0,Math.PI*2);ctx.stroke()}if(o.front_link){ctx.fillStyle=C.green;ctx.font='bold 10px Arial';ctx.fillText('+FRONT',p[0]+p[2]*.62,p[1]+12)}if(o.camera_confirmed&&o.source!=='c4_camera'){ctx.strokeStyle=C.camera;ctx.lineWidth=2.2;ctx.beginPath();ctx.arc(p[0],p[1],Math.max(p[2],p[3])*.82,0,Math.PI*2);ctx.stroke();ctx.fillStyle=C.camera;ctx.font='bold 9px Arial';ctx.fillText('CAM',p[0]+p[2]*.62,p[1]+24)}if(o.shadow_role){ctx.fillStyle=o.shadow_role==='L1'?C.gold:C.orange;ctx.font='bold 10px Arial';ctx.fillText(o.shadow_role,p[0]+p[2]*.62,p[1]-16)}if(o.vx!=null){ctx.fillStyle=col;ctx.font='10px Arial';ctx.fillText(`${x.toFixed(1)}m ${Number(o.vx).toFixed(1)}m/s`,p[0]+p[2]*.62,p[1]+10)}if((mode==='shadow'||rangeMode==='drive')&&o.road_d!=null){ctx.fillStyle=C.muted;ctx.font='9px Arial';ctx.fillText(`d ${Number(o.road_d).toFixed(2)}m`,p[0]+p[2]*.62,p[1]+22)}}drawTeacherMarkers()}
function side(){if(!state)return;const now=performance.now();if(now-lastSideUpdate<180)return;lastSideUpdate=now;const cs=state.corner_fused_objects||[],fs=state.front_objects||[],sp=state.standard_front_preview||[],sps=state.standard_front_preview_stats||{},as=state.corner_front_associations||[],cams=state.camera_leads||[],cm=state.camera_fusion_matches||[],sh=state.shadow_leads||{},ss=sh.stats||{},lg=state.shadow_logger||{},raws=state.raw_objects||[],cst=state.camera_fusion_stats||{},t=state.scc_teacher||{},m=state.scc_front_match||{},diag=state.diagnostics||{},rm=state.road_model||{};const sccRx=(t.bus!==undefined)?`RX bus${t.bus}`:'NO 0x1A0 RX';const py40=rm.path_y_40m==null?'--':Number(rm.path_y_40m).toFixed(2)+'m';summary.innerHTML=`<div class="metric"><span>Mode</span><b>${mode.toUpperCase()} / ${rangeMode==='wide'?'WIDE':(rangeMode==='drive'?'DRIVE 3-LANE':rangeMode.toUpperCase()+' 1:1')}</b></div><div class="metric"><span>C4 road model</span><b class="${rm.fresh?'fusion':'bad'}">${rm.fresh?'FRESH':'STALE'} ${rm.age_ms==null?'':Number(rm.age_ms).toFixed(0)+'ms'}</b></div><div class="metric"><span>Curve / path@40m</span><b class="fusion">${rm.curve_direction||'--'} / ${py40}</b></div><div class="metric"><span>Lane lines / horizon</span><b>${rm.confident_lane_lines??0}/${(rm.lane_lines||[]).length} / ${rm.path_x_max_m==null?'--':Number(rm.path_x_max_m).toFixed(0)+'m'}</b></div><div class="metric"><span>Corner fused</span><b class="fusion">${cs.length}</b></div><div class="metric"><span>Front ref tracks</span><b class="front">${fs.length}</b></div><div class="metric"><span>STD preview</span><b class="teacher">${sp.length}</b></div><div class="metric"><span>G1 raw→dedup / invalid</span><b>${sps.group1_fresh_before_dedup??0}→${sps.group1_after_dedup??0} / ${sps.group1_invalid_sentinel_rejected??0}</b></div><div class="metric"><span>Preview C/R/Cand</span><b>${sps.confirmed_count??0}/${sps.corroborated_count??0}/${sps.candidate_count??0}</b></div><div class="metric"><span>Preview ref/CAM</span><b>${sps.reference_match_count??0}/${sps.camera_match_count??0}</b></div><div class="metric"><span>Corner↔Front</span><b>${as.length}</b></div><div class="metric"><span>C4 camera leads</span><b style="color:${C.camera}">${cams.length}</b></div><div class="metric"><span>CAM↔Radar</span><b style="color:${C.camera}">${cm.length}</b></div><div class="metric"><span>RAW objects</span><b style="color:#3e9fff">${raws.length}</b></div><div class="metric"><span>Shadow L1/L2</span><b>${sh.leadOne?.status?(sh.leadOne.key||'L1'):'--'} / ${sh.leadTwo?.status?(sh.leadTwo.key||'L2'):'--'}</b></div><div class="metric"><span>CONTROL</span><b class="bad">MONITOR ONLY</b></div>`;const names=rangeMode==='drive'?['left1','ego','right1']:['left2','left1','ego','right1','right2'];zonesEl.innerHTML=names.map(n=>`<div class="zone ${(state.zones?.[n]?.occupied)?'on':''}">${n.replace('left','L').replace('right','R').toUpperCase()}</div>`).join('');const rearAll=state.teacher_rear||[];const rearText=rearAll.length?rearAll.map(x=>`${x.sector} ${Number(x.distance_candidate_m).toFixed(1)}m S${x.status_raw}${x.teacher_usable?'✓':''}`).join(' / '):'NO 0x1EA RX';const rawDist=(t.distance_m!==undefined)?Number(t.distance_m).toFixed(1)+'m':'--';const rawV=(t.rel_speed_mps!==undefined)?Number(t.rel_speed_mps).toFixed(1)+'m/s':'--';const flags=(t.bus!==undefined)?`B${t.bus} M${t.main_mode_acc??'?'} A${t.acc_mode??'?'} V${t.obj_valid_raw??'?'} S${t.scc_obj_sta??'?'}`:'--';teachers.innerHTML=`<div class="metric"><span>Rear 0x1EA</span><b class="${rearAll.length?'rear':'bad'}">${rearText}</b></div><div class="metric"><span>SCC 0x1A0</span><b class="${t.bus!==undefined?'teacher':'bad'}">${sccRx}</b></div><div class="metric"><span>SCC raw</span><b>${rawDist} / ${rawV}</b></div><div class="metric"><span>flags</span><b>${flags}</b></div><div class="metric"><span>strict usable</span><b class="${t.teacher_usable?'teacher':'dim'}">${t.teacher_usable?'YES':'NO'}</b></div><div class="metric"><span>match</span><b class="teacher">${m.confirmed?'CONF '+(m.front_key||''):(m.matched?'SEARCH':'--')}</b></div><div style="margin-top:7px;font-size:10px"><b style="color:${C.FL}">■ FL</b> &nbsp;<b style="color:${C.FR}">■ FR</b> &nbsp;<b style="color:${C.RL}">■ RL</b> &nbsp;<b style="color:${C.RR}">■ RR</b><br><span class="dim">corner 색상은 위치 sector 추정(센서 ownership 확정 아님)</span></div><hr style="border:0;border-top:1px solid #24404f;margin:8px 0"><div class="metric"><span>Shadow leadOne</span><b class="teacher">${sh.leadOne?.status?`${sh.leadOne.key} ${Number(sh.leadOne.dRel).toFixed(1)}m`:'--'}</b></div><div class="metric"><span>Shadow leadTwo</span><b style="color:${C.orange}">${sh.leadTwo?.status?`${sh.leadTwo.key} ${Number(sh.leadTwo.dRel).toFixed(1)}m`:'--'}</b></div><div class="metric"><span>path age</span><b>${ss.path_age_ms==null?'--':Number(ss.path_age_ms).toFixed(0)+'ms'}</b></div><div class="metric"><span>Shadow log</span><b style="color:${lg.enabled?C.green:C.muted}">${lg.enabled?`ON ${lg.records||0}`:'OFF'}</b></div><div class="metric"><span>radarTracks TX</span><b class="bad">OFF</b></div>`;objectsEl.innerHTML=arr().slice().sort((a,b)=>Number(a.x)-Number(b.x)).slice(0,24).map(o=>`<div class="obj"><span class="id" style="color:${srcColor(o)}">${o.vehicle_key||o.key||'?'}</span><span>${o.sector||o.front_sector||''}</span><span>${Number(o.x).toFixed(1)}m ${o.vx==null?'':Number(o.vx).toFixed(1)+'m/s'}${o.road_lane?` <span class="badge bRoad">${String(o.road_lane).toUpperCase()} d=${Number(o.road_d).toFixed(1)}</span>`:(o.road_lane_source==='c4_path_out_of_range'?'<span class="badge bRoad">PATH OUT</span>':'')}${o.preview_quality?`<span class="badge bLink">${o.preview_quality}</span>`:''}${o.scc_teacher_confirmed?'<span class="badge bScc">SCC</span>':''}${o.teacher_match?'<span class="badge bRear">LR/RR</span>':''}${o.front_link?'<span class="badge bLink">+FRONT</span>':''}${o.camera_confirmed?'<span class="badge bCam">CAM</span>':''}${(o.vehicle_duplicates_merged||0)>0?`<span class="badge bLink">MERGE×${Number(o.vehicle_member_count||1)}</span>`:''}${o.shadow_role==='L1'?'<span class="badge bL1">L1</span>':''}${o.shadow_role==='L2'?'<span class="badge bL2">L2</span>':''}${o.cutin_confirmed?'<span class="badge bL2">CUT-IN</span>':''}${mode==='raw'&&o.raw_address!=null?`<span class="badge" style="background:#153d5b;color:#cdeaff">0x${Number(o.raw_address).toString(16).toUpperCase()}</span>`:''}</span></div>`).join('')}
async function poll(){try{const r=await fetch('/state',{cache:'no-store'});state=await r.json();dirty=true;side();requestAnimationFrame(draw)}catch(e){}setTimeout(poll,100)}document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{mode=b.dataset.mode;document.querySelectorAll('[data-mode]').forEach(x=>x.classList.toggle('active',x===b));dirty=true;side();requestAnimationFrame(draw)});document.querySelectorAll('[data-range]').forEach(b=>b.onclick=()=>{rangeMode=b.dataset.range;document.querySelectorAll('[data-range]').forEach(x=>x.classList.toggle('active',x===b));buildBg();dirty=true;side();requestAnimationFrame(draw)});document.getElementById('fs').onclick=async()=>{try{if(!document.fullscreenElement)await document.documentElement.requestFullscreen();else await document.exitFullscreen()}catch(e){}};poll();
</script></body></html>
'''

class H(BaseHTTPRequestHandler):
  def log_message(self,*a): pass
  def do_GET(self):
    if self.path.split('?')[0]=='/state':
      mark_browser_active()
      with state_lock: b=json.dumps(latest_state,separators=(',',':')).encode()
      self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    else:
      b=HTML.encode();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)

def http_thread(): ThreadingHTTPServer(('0.0.0.0',HTTP_PORT),H).serve_forever()
def atomic_write(path,text): tmp=path.with_suffix('.tmp');tmp.write_text(text);tmp.replace(path)

def main():
  threading.Thread(target=http_thread,daemon=True).start()
  can_sock=messaging.sub_sock('can',timeout=0,conflate=False)
  model_sock=messaging.sub_sock('modelV2',timeout=0,conflate=True)
  carstate_sock=messaging.sub_sock('carState',timeout=0,conflate=True)
  radarstate_sock=messaging.sub_sock('radarState',timeout=0,conflate=True)
  tracks=TrackStore(ttl_s=.75,continuity_s=.30);corner_fuser=CornerFusionTracker();corner_vehicle_tracker=VehicleFootprintTracker('VC');scc_matcher=SccFrontTeacherMatcher();camera_fuser=CameraRadarFusion();shadow_verifier=ShadowLeadVerifier();shadow_logger=ShadowLogger();front_preview=StandardFrontPreview();camera_leads=[];model_path=[];road_model={};model_path_recv_ns=0;v_ego=0.0;v_ego_recv_ns=0;production=None;rear_teacher=[];rear_teacher_by_bus={};scc_teacher=None;scc_teacher_by_bus={};SCC_BUS=int(os.getenv('G80_SCC_BUS',str(DEFAULT_SCC_BUS)))
  udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
  diag={'corner_A_frames':0,'corner_B_frames':0,'corner_decoded_total':0,'corner_rear_total':0,'corner_front_total':0,'corner_A_by_bus':{},'corner_B_by_bus':{},'scc_frames_by_bus':{},'scc_teacher_updates':0,'model_frames':0,'camera_leads_latest':0,'model_transport_lag_ms':None,'last_transport_lag_ms':None}
  next_pub=time.monotonic();next_debug_write=0.0
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
      model_path=path_as_tuples(road_model)
      model_path_recv_ns=mrecv_ns
      diag['model_frames']+=1
      diag['camera_leads_latest']=len(camera_leads)
      diag['model_transport_lag_ms']=round((mrecv_ns-int(mmsg.logMonoTime))/1e6,3)

    csmsg=messaging.recv_one_or_none(carstate_sock)
    if csmsg is not None:
      try:
        v_ego=float(csmsg.carState.vEgo)
        v_ego_recv_ns=time.monotonic_ns()
      except Exception: pass

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
      raw=tracks.snapshot(now_ns);filt=filtered_objects(raw,rear_teacher);cf=corner_fuser.update(filt,now_ns);fronts=build_front_objects(filt);fronts,scc_match_status=scc_matcher.update(fronts,scc_teacher,now_ns);combined=associate_corner_front(cf['corner_fused_objects'],fronts);corners=combined['corner_objects'];fronts=combined['front_objects'];radar_all_fused=combined['all_fused_objects'];assocs=combined['associations'];camf=camera_fuser.update(radar_all_fused,camera_leads,now_ns,fronts);sensor_fused=camf['sensor_fused_objects'];front_sensor=camf['front_sensor_objects'];corner_vehicle,corner_vehicle_stats=corner_vehicle_tracker.update(corners,now_ns);stdp=front_preview.update(raw,fronts,camera_leads,scc_teacher,now_ns);shadow=shadow_verifier.update(sensor_fused,model_path,v_ego,now_ns,production,model_path_recv_ns,v_ego_recv_ns)
      road_view=road_model_with_age(road_model,now_ns)
      raw_view=annotate_objects(raw,road_view);filt_view=annotate_objects(filt,road_view);corner_vehicle_view=annotate_objects(corner_vehicle,road_view);corners_view=annotate_objects(corners,road_view);fronts_view=annotate_objects(fronts,road_view);radar_all_view=annotate_objects(radar_all_fused,road_view);sensor_fused_view=annotate_objects(sensor_fused,road_view);front_sensor_view=annotate_objects(front_sensor,road_view);std_points_view=annotate_objects(stdp['points'],road_view);camera_leads_view=annotate_objects(camera_leads,road_view)
      zones=occupied_zones(corner_vehicle_view)
      # Core state is always produced because radar fusion/teacher logic and the
      # Android UDP stream must remain live even with no browser connected.
      core={'version':21,'mono_ns':now_ns,'objects':sensor_fused_view,
            'sensor_fused_objects':sensor_fused_view,'all_fused_objects':sensor_fused_view,
            'radar_fused_objects':radar_all_view,
            'corner_fused_objects':corner_vehicle_view,'corner_radar_objects':corners_view,'front_objects':fronts_view,'front_sensor_objects':front_sensor_view,
            'standard_front_preview':std_points_view,'standard_front_preview_stats':stdp['stats'],
            'camera_leads':camera_leads_view,'camera_fusion_matches':camf['camera_matches'],
            'road_model':road_view,
            'camera_fusion_stats':dict(camf['stats'],corner_vehicle_objects_before=corner_vehicle_stats['vehicle_objects_before'],corner_vehicle_objects_after=corner_vehicle_stats['vehicle_objects_after'],corner_vehicle_duplicates_merged=corner_vehicle_stats['vehicle_duplicates_merged']),'shadow_leads':shadow,
            'corner_front_associations':assocs,'zones':zones,
            'teacher_rear':rear_teacher,'scc_teacher':scc_teacher or {},
            'scc_teacher_by_bus':{str(k):v for k,v in scc_teacher_by_bus.items()},
            'scc_front_match':scc_match_status,'corner_fusion_stats':cf['stats'],
            'shadow_logger':shadow_logger.status()}

      # Persistent shadow evaluation log under /data/radar (outside the git tree).
      # 5 Hz periodic by default, plus immediate records when L1/L2/CUT-IN state changes.
      shadow_logger.maybe_write(core,model_path,v_ego,model_path_recv_ns,v_ego_recv_ns,diag,now_ns)
      core['shadow_logger']=shadow_logger.status()

      # Android/render protocol stays active independently of the browser.
      try:
        udp.sendto(json.dumps(build_render_packet(core),separators=(',',':')).encode(),(UDP_HOST,UDP_PORT))
      except OSError:
        pass

      # Browser/debug serialization stays on-demand by default.
      # RAW/FILTERED remain in latest_state, so first /state already contains RAW.
      # Set G80_DEBUG_STATE_HZ>0 only for focused shared-memory logging.
      ui_active=browser_is_active(now_ns)
      debug_due=DEBUG_STATE_HZ>0.0 and now>=next_debug_write
      if ui_active or debug_due:
        out=dict(core)
        out.update({
          'corner_local_objects':cf['corner_local_objects'],
          'filtered_objects':filt_view,
          'raw_objects':raw_view,
          'diagnostics':dict(diag,ui_sleeping=not ui_active),
          'browser_active':ui_active,
          'notes':{
            'ui':'v21-monitor-only-perspective-road-corner-colors-front-preview-360',
            'display_scale':'SHORT/LONG 1:1 + WIDE + DRIVE perspective; C4 modelV2 curved road overlay',
            'browser_policy':'full JSON on browser poll; optional debug persistence',
            'udp':'compact Android final sensor-fusion packet on 28991',
            'control':'disabled',
            'integration':'NOT connected to RadarInterface/radarTracks/radard',
            'future_front':'plain-dict RadarPoint preview only',
            'road_geometry':'C4 modelV2 position/laneLines; horizon-limited Frenet + perspective display only',
            'shadow_control':'NEVER publishes radarTracks/radarState / NEVER CAN TX',
            'debug_state_hz':DEBUG_STATE_HZ,'shadow_log_dir':'/data/radar'
          }
        })
        try:
          atomic_write(STATE_PATH,json.dumps(out,separators=(',',':')))
        except OSError:
          pass
        if debug_due:
          next_debug_write=now+1.0/max(DEBUG_STATE_HZ,0.1)
        with state_lock:
          latest_state.clear();latest_state.update(out)
      else:
        compact=dict(core)
        compact['browser_active']=False
        compact['corner_local_objects']=cf['corner_local_objects']
        compact['filtered_objects']=filt_view
        compact['raw_objects']=raw_view
        compact['diagnostics']=dict(diag,ui_sleeping=True)
        with state_lock:
          latest_state.clear();latest_state.update(compact)
      next_pub=now+1.0/max(PUBLISH_HZ,1.0)
    if processed==0: time.sleep(.002)
if __name__=='__main__': main()
