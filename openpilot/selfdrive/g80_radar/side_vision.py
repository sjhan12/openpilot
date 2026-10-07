#!/usr/bin/env python3
from __future__ import annotations

# Keep OpenCV/BLAS from spawning a large thread pool on comma hardware.
import os
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('MKL_NUM_THREADS', '1')
os.environ.setdefault('VECLIB_MAXIMUM_THREADS', '1')
os.environ.setdefault('NUMEXPR_NUM_THREADS', '1')

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import site
import numpy as np

# Optional Python packages installed on /data are visible without touching the AGNOS system partition.
PYDEPS = os.getenv('G80_SIDE_VISION_PYDEPS', '/data/g80_pydeps')
if os.path.isdir(PYDEPS):
  site.addsitedir(PYDEPS)

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.cereal.visionipc import VisionStreamType
try:
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
except Exception:
  get_nv12_info = None
from openpilot.selfdrive.g80_radar.build_info import BUILD_VERSION, SIDE_VISION_API_VERSION
from openpilot.selfdrive.g80_radar.side_vision_inference import SideVisionInference, V_ASM_MODEL_PATH
from openpilot.selfdrive.g80_radar.side_vision_image import nv12_to_rgb, nv12_buffer_to_rgb, write_png_atomic
from openpilot.selfdrive.g80_radar.vision_cpu_throttle import device_cpu_throttle_factor

try:
  from openpilot.common.realtime import set_core_affinity
except Exception:
  set_core_affinity = None

SIDE_VISION_UDP_HOST = os.getenv('G80_SIDE_VISION_UDP_HOST', '127.0.0.1')
SIDE_VISION_UDP_PORT = int(os.getenv('G80_SIDE_VISION_UDP_PORT', '28993'))
SIDE_VISION_HTTP_HOST = os.getenv('G80_SIDE_VISION_HTTP_HOST', '0.0.0.0')
SIDE_VISION_HTTP_PORT = int(os.getenv('G80_SIDE_VISION_HTTP_PORT', '28994'))
CONFIG_PATH = Path(os.getenv('G80_SIDE_VISION_CONFIG', '/data/radar/g80_side_vision_config.json'))
SNAPSHOT_PATH = Path(os.getenv('G80_SIDE_VISION_SNAPSHOT', '/data/radar/g80_side_vision_snapshot.png'))
DISABLE_MARKER = Path(os.getenv('G80_SIDE_VISION_DISABLE_MARKER', '/data/radar/DISABLE_G80_SIDE_VISION'))

BASE_INTERVAL = float(os.getenv('G80_SIDE_VISION_BASE_INTERVAL', '1.0'))
FOLLOWUP_INTERVAL = float(os.getenv('G80_SIDE_VISION_FOLLOWUP_INTERVAL', '0.30'))
FOLLOWUP_WINDOW = float(os.getenv('G80_SIDE_VISION_FOLLOWUP_WINDOW', '1.0'))
PARAM_REFRESH_INTERVAL = 2.0
STATUS_INTERVAL = 0.25
SNAPSHOT_MAX_ATTEMPTS = max(1, int(os.getenv('G80_SIDE_VISION_SNAPSHOT_MAX_ATTEMPTS','3')))
MODEL_RETRY_INTERVAL = 5.0
AFFINITY_CORES = [0, 1, 2]
EXPECTED_MODEL_GIT_BLOB = '6a1ea709681ce256927e0cf36e53defad5ce94d8'
STAR_PILOT_MODEL_COMMIT = '0122e4069b627948b219e419d2e84b5f22773c43'


def _env_bool(name: str, default: bool = True) -> bool:
  raw = os.getenv(name)
  if raw is None:
    return default
  return str(raw).strip().lower() not in ('0', 'false', 'no', 'off', 'disable', 'disabled')


def _clamp(v, lo, hi, default):
  try:
    x = float(v)
    if not np.isfinite(x):
      return default
    return max(lo, min(hi, x))
  except Exception:
    return default


def _nv12_y_rows(width: int, height: int, stride: int) -> int:
  """Return physical Y-plane row count in comma VisionIPC NV12 buffers."""
  if get_nv12_info is not None:
    try:
      nv_stride, y_rows, _uv_rows, _size = get_nv12_info(int(width), int(height))
      # VisionIPC stride should match the helper. Prefer the helper's padding even
      # if a future camera exposes a wider DMA stride.
      if int(nv_stride) <= int(stride):
        return int(y_rows)
    except Exception:
      pass
  # Qualcomm Venus buffers align luma height to 32 rows. This fallback is safe
  # for current comma cabin cameras; compact buffers simply resolve to height.
  aligned = ((int(height) + 31) // 32) * 32
  return aligned if aligned < 2 * int(height) else int(height)


def _atomic_json(path: Path, value: dict):
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix(path.suffix + '.tmp')
  tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')
  os.replace(tmp, path)


SETUP_HTML = r'''<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>G80 V51 SIDE VISION</title><style>
body{margin:0;background:#071018;color:#e8f4fa;font-family:Arial,"Noto Sans KR",sans-serif}.wrap{max-width:1050px;margin:auto;padding:18px}.card{background:#0b1c27;border:1px solid #345164;border-radius:12px;padding:14px;margin-bottom:12px}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}button,input{background:#112b3c;color:#e8f4fa;border:1px solid #4b7288;border-radius:8px;padding:9px 11px}button.active{background:#186ab2}.ok{color:#61e29a}.warn{color:#ffd266}.bad{color:#ff7183}canvas{width:100%;height:auto;border:1px solid #456476;border-radius:8px;background:#02070a;cursor:crosshair}.small{font-size:12px;color:#a9c1ce;line-height:1.5}.status{font-family:monospace;font-size:12px;white-space:pre-wrap}.pill{display:inline-block;padding:3px 8px;border:1px solid #416276;border-radius:999px;margin:2px}.left{color:#6fd7ff}.right{color:#ff93c4}</style></head><body><div class="wrap">
<div class="card"><h2>G80 V51 · SIDE VISION SETUP</h2><div class="small">StarPilot V-ASM 방식을 G80용으로 분리 이식한 SHADOW 센서입니다. FutureGap/조향 제어에는 직접 연결하지 않습니다. 아래 LEFT는 차량 좌측(운전석 쪽), RIGHT는 차량 우측(조수석 쪽)을 뜻합니다. cabin raw image는 화면상 좌우가 거울처럼 보이지 않을 수 있으므로 화면 위치가 아니라 실제 차량 좌/우 창문 기준으로 지정하십시오.</div><div id="status" class="status"></div></div>
<div class="card"><div class="row"><button id="refresh">SNAPSHOT 새로 요청</button><button data-side="left">LEFT 창문 지정</button><button data-side="right">RIGHT 창문 지정</button><button id="undo">UNDO</button><button id="clear">CLEAR</button><button id="save">SAVE</button><button id="delete">CONFIG DELETE</button></div><div class="row" style="margin-top:8px"><label>confidence <input id="conf" type="number" min="0.80" max="1.00" step="0.01" value="0.94"></label><label>smoothing(s) <input id="smooth" type="number" min="0.01" max="0.50" step="0.01" value="0.20"></label></div><div class="small" style="margin-top:8px">창문 유리 영역을 3점 이상 클릭하십시오. A/B pillar와 실내는 가급적 제외합니다. 저장 뒤 daemon이 자동으로 config를 다시 읽습니다.</div></div>
<div class="card"><canvas id="cv"></canvas></div></div><script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d'),statusEl=document.getElementById('status');let img=null,side=null,left=[],right=[],nativeW=0,nativeH=0;
function draw(){if(!img)return;ctx.clearRect(0,0,cv.width,cv.height);ctx.drawImage(img,0,0,cv.width,cv.height);drawPoly(left,'#67d7ff','LEFT');drawPoly(right,'#ff80bd','RIGHT')}
function drawPoly(p,c,t){if(!p.length)return;ctx.strokeStyle=c;ctx.fillStyle=c+'33';ctx.lineWidth=3;ctx.beginPath();p.forEach((q,i)=>i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1]));if(p.length>=3){ctx.closePath();ctx.fill()}ctx.stroke();ctx.fillStyle=c;ctx.font='bold 15px Arial';p.forEach(q=>{ctx.beginPath();ctx.arc(q[0],q[1],5,0,Math.PI*2);ctx.fill()});ctx.fillText(t,p[0][0]+7,p[0][1]-7)}
async function loadConfig(){try{const r=await fetch('/config',{cache:'no-store'});const c=await r.json();document.getElementById('conf').value=c.confidence_threshold??0.94;document.getElementById('smooth').value=c.smooth_sec??0.20;if(img&&c.width&&c.height){const sx=cv.width/c.width,sy=cv.height/c.height;left=(c.poly_left||[]).map(q=>[q[0]*sx,q[1]*sy]);right=(c.poly_right||[]).map(q=>[q[0]*sx,q[1]*sy]);draw()}}catch(e){}}
async function loadSnapshot(retry=true){try{const r=await fetch('/snapshot?x='+Date.now(),{cache:'no-store'});if(r.status===202){if(retry)setTimeout(()=>loadSnapshot(true),700);return}if(!r.ok)return;const b=await r.blob(),u=URL.createObjectURL(b),im=new Image();im.onload=()=>{img=im;nativeW=im.naturalWidth;nativeH=im.naturalHeight;cv.width=Math.min(nativeW,1200);cv.height=Math.round(cv.width*nativeH/nativeW);URL.revokeObjectURL(u);loadConfig();draw()};im.src=u}catch(e){}}
cv.onclick=e=>{if(!img||!side)return;const r=cv.getBoundingClientRect(),x=(e.clientX-r.left)*cv.width/r.width,y=(e.clientY-r.top)*cv.height/r.height;(side==='left'?left:right).push([x,y]);draw()};
document.querySelectorAll('[data-side]').forEach(b=>b.onclick=()=>{side=b.dataset.side;document.querySelectorAll('[data-side]').forEach(x=>x.classList.toggle('active',x===b))});
document.getElementById('undo').onclick=()=>{const p=side==='right'?right:left;p.pop();draw()};document.getElementById('clear').onclick=()=>{if(side==='left')left=[];else if(side==='right')right=[];else{left=[];right=[]}draw()};
document.getElementById('refresh').onclick=async()=>{await fetch('/request_snapshot',{method:'POST'});setTimeout(()=>loadSnapshot(true),600)};
document.getElementById('save').onclick=async()=>{if(!img||(!left.length&&!right.length)){alert('snapshot과 polygon이 필요합니다');return}const sx=nativeW/cv.width,sy=nativeH/cv.height,sc=p=>p.map(q=>[Math.round(q[0]*sx),Math.round(q[1]*sy)]);const c={width:nativeW,height:nativeH,poly_left:left.length>=3?sc(left):[],poly_right:right.length>=3?sc(right):[],confidence_threshold:Number(document.getElementById('conf').value),smooth_sec:Number(document.getElementById('smooth').value)};const r=await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});alert(r.ok?'저장 완료':'저장 실패')};
document.getElementById('delete').onclick=async()=>{await fetch('/config',{method:'DELETE'});left=[];right=[];draw()};
async function poll(){try{const r=await fetch('/state',{cache:'no-store'}),s=await r.json();const L=s.left||{},R=s.right||{};const ready=!!(s.model_valid&&s.config_loaded&&s.onroad);const st=!s.model_valid?'MODEL WAIT':(!s.config_loaded?'CONFIG WAIT':(!s.onroad?'OFFROAD':(!s.camera_connected?'CAMERA WAIT':'READY')));const lf=ready?`active=${L.active?'1':'0'} raw=${Number(L.raw_confidence||0).toFixed(3)} score=${Number(L.score||0).toFixed(3)}`:'active=-- raw=-- score=--';const rf=ready?`active=${R.active?'1':'0'} raw=${Number(R.raw_confidence||0).toFixed(3)} score=${Number(R.score||0).toFixed(3)}`:'active=-- raw=-- score=--';statusEl.innerHTML=`state=${st} model=${s.model_valid?'OK':'MISSING/ERROR'} backend=${s.inference_backend||'none'} cv2=${s.cv2_available?'YES':'NO'} config=${s.config_loaded?'OK':'NEEDED'} camera=${s.camera_connected?'CONNECTED':'WAIT'} snapshot=${s.snapshot_available?'OK':(s.snapshot_pending?'PENDING':'NONE')} frames=${s.frames_received||0} ${s.camera_width||0}x${s.camera_height||0} onroad=${s.onroad?'1':'0'} throttle=${Number(s.throttle_factor||1).toFixed(2)}x\n<span class="left">LEFT ${lf}</span>  <span class="right">RIGHT ${rf}</span>\nmodel_error=${s.model_error||'-'}\nsnapshot_error=${s.snapshot_error||'-'}\n${s.last_error||''}`;}catch(e){}setTimeout(poll,1000)}
loadSnapshot(true);poll();
</script></body></html>'''


class SideVisionDaemon:
  def __init__(self):
    from msgq.visionipc import VisionIpcClient

    self.VisionIpcClient = VisionIpcClient
    self.stream_type = VisionStreamType.VISION_STREAM_CABIN
    self.client = None
    self.sm = messaging.SubMaster(['deviceState', 'carState'])
    self.params = Params()
    self.inference = SideVisionInference(V_ASM_MODEL_PATH)
    self.model_last_try = 0.0
    self.config = {}
    self.config_mtime_ns = -1
    self.config_loaded = False
    self.current_side = 'left'
    self.last_inference_at = 0.0
    self.last_inference_at_side = {'left': 0.0, 'right': 0.0}
    self.last_inference_mono_ns = 0
    self.followup_until = 0.0
    self.inference_ms = 0.0
    self.throttle_factor = 1.0
    self.current_interval = BASE_INTERVAL
    self.camera_connected = False
    self.onroad = False
    self.last_error = ''
    self.last_status_send = 0.0
    self.last_status_log = 0.0
    self.snapshot_request = threading.Event()
    self.snapshot_lock = threading.Lock()
    self.snapshot_available = SNAPSHOT_PATH.is_file()
    self.snapshot_requested_count = 0
    self.snapshot_attempt_count = 0
    self.snapshot_success_count = 0
    self.snapshot_last_error = ''
    self.snapshot_last_ms = 0.0
    self.frames_received = 0
    self.last_frame_mono_ns = 0
    self.last_frame_id = -1
    self.camera_width = 0
    self.camera_height = 0
    self.camera_stride = 0
    self.camera_last_error = ''
    self.auto_snapshot_requested = False
    self._snapshot_driver_view_owned = False
    self._snapshot_driver_view_started_at = 0.0
    self.enabled_env = _env_bool('G80_SIDE_VISION_ENABLE', True)
    self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._maybe_load_model(force=True)
    self._reload_config(force=True)
    self._start_http()
    self._set_affinity()

  def _set_affinity(self):
    if set_core_affinity is None:
      return
    try:
      set_core_affinity(AFFINITY_CORES)
    except Exception as e:
      self.last_error = f'affinity: {e!r}'

  def _maybe_load_model(self, force=False):
    now = time.monotonic()
    if self.inference.valid:
      return
    if not force and now - self.model_last_try < MODEL_RETRY_INTERVAL:
      return
    self.model_last_try = now
    self.inference.load()

  def _reload_config(self, force=False):
    try:
      mtime = CONFIG_PATH.stat().st_mtime_ns if CONFIG_PATH.exists() else 0
    except OSError:
      mtime = 0
    if not force and mtime == self.config_mtime_ns:
      return
    self.config_mtime_ns = mtime
    cfg = {}
    try:
      if CONFIG_PATH.exists():
        cfg = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
      if not isinstance(cfg, dict):
        cfg = {}
    except Exception as e:
      self.last_error = f'config: {e!r}'
      cfg = {}
    self.config = cfg
    try:
      self.inference.reset_state()
      self.inference.load_config(cfg)
      self.config_loaded = bool(self.inference.configured_sides)
    except Exception as e:
      self.config_loaded = False
      self.last_error = f'config load: {e!r}'

  def _start_http(self):
    owner = self

    class Handler(BaseHTTPRequestHandler):
      def log_message(self, *args):
        pass

      def _send_json(self, obj, code=200):
        b = json.dumps(obj, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
        self.send_response(code); self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)

      def _path(self):
        return urlparse(self.path).path

      def do_GET(self):
        p = self._path()
        if p == '/':
          b = SETUP_HTML.encode('utf-8'); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(b))); self.end_headers(); self.wfile.write(b); return
        if p in ('/state', '/health'):
          self._send_json(owner.status()); return
        if p == '/config':
          self._send_json(owner.config); return
        if p == '/snapshot':
          if SNAPSHOT_PATH.is_file():
            try:
              b = SNAPSHOT_PATH.read_bytes(); self.send_response(200); self.send_header('Content-Type','image/png' if SNAPSHOT_PATH.suffix.lower()=='.png' else 'application/octet-stream'); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(b))); self.end_headers(); self.wfile.write(b); return
            except OSError:
              pass
          owner.snapshot_requested_count += 1; owner.snapshot_request.set(); self._send_json({'pending': True, 'message': 'Waiting for next driver-camera frame'}, 202); return
        self._send_json({'error':'not found'},404)

      def do_POST(self):
        p = self._path()
        if p == '/request_snapshot':
          owner.snapshot_requested_count += 1; owner.snapshot_request.set(); self._send_json({'ok':True,'request_count':owner.snapshot_requested_count}); return
        if p == '/config':
          try:
            n = min(int(self.headers.get('Content-Length','0') or 0), 128000)
            data = json.loads(self.rfile.read(n).decode('utf-8'))
            if not isinstance(data, dict): raise ValueError('config must be object')
            if int(data.get('width',0) or 0) <= 0 or int(data.get('height',0) or 0) <= 0: raise ValueError('invalid image size')
            pl = data.get('poly_left',[]) or []; pr = data.get('poly_right',[]) or []
            if len(pl) < 3 and len(pr) < 3: raise ValueError('at least one polygon needs 3+ points')
            data['confidence_threshold'] = _clamp(data.get('confidence_threshold',0.94),0.80,1.00,0.94)
            data['smooth_sec'] = _clamp(data.get('smooth_sec',0.20),0.01,0.50,0.20)
            _atomic_json(CONFIG_PATH,data); owner._reload_config(force=True); self._send_json({'ok':True,'config':owner.config}); return
          except Exception as e:
            self._send_json({'error':str(e)},400); return
        self._send_json({'error':'not found'},404)

      def do_DELETE(self):
        if self._path() == '/config':
          try:
            CONFIG_PATH.unlink(missing_ok=True)
          except Exception:
            pass
          owner._reload_config(force=True); self._send_json({'ok':True}); return
        self._send_json({'error':'not found'},404)

    def serve():
      try:
        ThreadingHTTPServer((SIDE_VISION_HTTP_HOST, SIDE_VISION_HTTP_PORT), Handler).serve_forever()
      except Exception as e:
        owner.last_error = f'http: {e!r}'

    threading.Thread(target=serve, name='g80-side-vision-http', daemon=True).start()

  def _connect_camera(self) -> bool:
    try:
      if self.client is not None and self.client.is_connected():
        self.camera_connected = True
        return True
      available = self.VisionIpcClient.available_streams('camerad', block=False)
      if self.stream_type not in available:
        self.camera_connected = False
        return False
      if self.client is None:
        self.client = self.VisionIpcClient('camerad', self.stream_type, True)
      if not self.client.is_connected():
        self.client.connect(True)
      self.camera_connected = bool(self.client.is_connected())
      return self.camera_connected
    except Exception as e:
      self.camera_connected = False
      self.camera_last_error = f'{e!r}'
      self.last_error = f'camera connect: {e!r}'
      return False

  def _ensure_driver_view_for_snapshot(self, now: float):
    """Start cabin camera while parked only for the annotation snapshot."""
    if self.onroad or self._snapshot_driver_view_owned:
      return
    try:
      if not self.params.get_bool('IsDriverViewEnabled'):
        self.params.put_bool('IsDriverViewEnabled', True)
        self._snapshot_driver_view_owned = True
        self._snapshot_driver_view_started_at = float(now)
    except Exception as e:
      self.last_error = f'driver-view snapshot start: {e!r}'

  def _release_driver_view_for_snapshot(self):
    if not self._snapshot_driver_view_owned:
      return
    try:
      self.params.remove('IsDriverViewEnabled')
    except Exception as e:
      self.last_error = f'driver-view snapshot stop: {e!r}'
    self._snapshot_driver_view_owned = False
    self._snapshot_driver_view_started_at = 0.0

  def _capture_snapshot(self, buffer):
    t0 = time.perf_counter_ns()
    self.snapshot_attempt_count += 1
    try:
      width = int(self.client.width); height = int(self.client.height); stride = int(self.client.stride)
      uv_offset = int(getattr(buffer, 'uv_offset', 0) or 0)
      if uv_offset <= 0:
        y_rows = _nv12_y_rows(width, height, stride)
        uv_offset = int(y_rows * stride)
      rgb = nv12_buffer_to_rgb(buffer.data, width, height, stride, uv_offset)
      write_png_atomic(SNAPSHOT_PATH, rgb)
      self.snapshot_available = True
      self.snapshot_success_count += 1
      self.snapshot_last_error = ''
      self.snapshot_request.clear()
      self._release_driver_view_for_snapshot()
    except Exception as e:
      self.snapshot_last_error = repr(e)
      self.last_error = f'snapshot: {e!r}'
      # Do not spin on a permanently failing full-frame RGB/PNG conversion.
      # Retry a few fresh frames, then clear the request and wait for a new click.
      if self.snapshot_attempt_count >= SNAPSHOT_MAX_ATTEMPTS:
        self.snapshot_request.clear()
        self._release_driver_view_for_snapshot()
        self.last_error = f'snapshot aborted after {self.snapshot_attempt_count} attempts: {e!r}'
    finally:
      self.snapshot_last_ms = (time.perf_counter_ns() - t0) / 1e6


  def _config_values(self):
    conf = _clamp(self.config.get('confidence_threshold',0.94),0.80,1.00,0.94)
    smooth = _clamp(self.config.get('smooth_sec',0.20),0.01,0.50,0.20)
    return conf, smooth, max(0.0, conf - 0.15)

  def _inference_interval(self, now: float) -> float:
    in_followup = now < self.followup_until
    base = FOLLOWUP_INTERVAL if in_followup else BASE_INTERVAL
    cpu = []
    try:
      if self.sm.valid.get('deviceState', False):
        cpu = list(self.sm['deviceState'].cpuUsagePercent)
    except Exception:
      pass
    self.throttle_factor = device_cpu_throttle_factor(cpu, name='g80-side-vision', cores=AFFINITY_CORES)
    self.current_interval = max(0.15, base * self.throttle_factor)
    return self.current_interval

  def status(self) -> dict:
    enabled = bool(self.enabled_env and not DISABLE_MARKER.exists())
    return {
      'api_version': SIDE_VISION_API_VERSION,
      'build': BUILD_VERSION,
      'source': 'G80_V51_SIDE_VISION',
      'fusion_mode': 'SHADOW_ONLY',
      'enabled': enabled,
      'model_valid': bool(self.inference.valid),
      'inference_ready': bool(enabled and self.onroad and self.config_loaded and self.inference.valid),
      'inference_backend': str(getattr(self.inference, 'backend', 'none')),
      'cv2_available': bool(getattr(self.inference, 'cv2_available', False)),
      'model_path': str(self.inference.model_path),
      'model_expected_git_blob': EXPECTED_MODEL_GIT_BLOB,
      'model_source_commit': STAR_PILOT_MODEL_COMMIT,
      'config_loaded': bool(self.config_loaded),
      'configured_sides': list(self.inference.configured_sides),
      'config_path': str(CONFIG_PATH),
      'snapshot_available': bool(SNAPSHOT_PATH.is_file()),
      'snapshot_path': str(SNAPSHOT_PATH),
      'snapshot_pending': bool(self.snapshot_request.is_set()),
      'snapshot_requested_count': int(self.snapshot_requested_count),
      'snapshot_attempt_count': int(self.snapshot_attempt_count),
      'snapshot_success_count': int(self.snapshot_success_count),
      'snapshot_last_ms': round(float(self.snapshot_last_ms), 2),
      'snapshot_error': str(self.snapshot_last_error),
      'camera_connected': bool(self.camera_connected),
      'camera_width': int(self.camera_width),
      'camera_height': int(self.camera_height),
      'camera_stride': int(self.camera_stride),
      'frames_received': int(self.frames_received),
      'last_frame_id': int(self.last_frame_id),
      'last_frame_mono_ns': int(self.last_frame_mono_ns),
      'camera_error': str(self.camera_last_error),
      'model_error': str(self.inference.last_error),
      'onroad': bool(self.onroad),
      'last_inference_mono_ns': int(self.last_inference_mono_ns),
      'inference_ms': round(float(self.inference_ms), 2),
      'inference_interval_s': round(float(self.current_interval), 3),
      'throttle_factor': round(float(self.throttle_factor), 3),
      'left': {'active': bool(self.inference.left_active), 'raw_confidence': round(float(self.inference.left_confidence),4), 'score': round(float(self.inference.left_score),4)},
      'right': {'active': bool(self.inference.right_active), 'raw_confidence': round(float(self.inference.right_confidence),4), 'score': round(float(self.inference.right_score),4)},
      'last_error': str(self.last_error or ''),
      'setup_url_port': SIDE_VISION_HTTP_PORT,
      'note': 'Camera evidence is logged/displayed only in V51; it does not modify FutureGap or vehicle control.',
      'packet_mono_ns': time.monotonic_ns(),
    }

  def _send_status(self, force=False):
    now = time.monotonic()
    if not force and now - self.last_status_send < STATUS_INTERVAL:
      return
    self.last_status_send = now
    try:
      payload = json.dumps(self.status(), separators=(',', ':'), allow_nan=False).encode('utf-8')
      self.udp.sendto(payload, (SIDE_VISION_UDP_HOST, SIDE_VISION_UDP_PORT))
    except Exception as e:
      self.last_error = f'udp status: {e!r}'

  def run(self):
    last_param_refresh = 0.0
    while True:
      loop_t0 = time.monotonic()
      try:
        self.sm.update(0)
        now = time.monotonic()
        if now - last_param_refresh >= PARAM_REFRESH_INTERVAL:
          last_param_refresh = now
          self._reload_config()
          self._maybe_load_model()

        try:
          self.onroad = bool(self.sm['deviceState'].started) if self.sm.valid.get('deviceState',False) else False
        except Exception:
          self.onroad = False

        enabled = bool(self.enabled_env and not DISABLE_MARKER.exists())
        need_snapshot = bool(self.snapshot_request.is_set() or (self.onroad and not self.config_loaded and not SNAPSHOT_PATH.is_file() and not self.auto_snapshot_requested))
        if need_snapshot:
          self.auto_snapshot_requested = True
          self.snapshot_request.set()
          if not self.onroad:
            self._ensure_driver_view_for_snapshot(now)

        # Snapshot setup may temporarily request the cabin stream while parked.
        # Normal inference remains strictly onroad.
        need_camera = bool(need_snapshot or (self.onroad and enabled and self.config_loaded and self.inference.valid))
        if not need_camera:
          # Do not leave a stale CONNECTED state from a prior snapshot/client.
          # If inference is not runnable (MODEL/CONFIG/OFF), report camera as inactive.
          self.camera_connected = False
          if not self.onroad:
            self.inference.reset_state()
            self.followup_until = 0.0
            self._release_driver_view_for_snapshot()
          self._send_status()
          time.sleep(0.05)
          continue

        if not self._connect_camera():
          self._send_status()
          time.sleep(0.08)
          continue

        # Snapshot requests use a bounded blocking receive. This is more reliable
        # during offroad camerad startup than polling only with timeout_ms=0.
        buffer = None
        if self.snapshot_request.is_set():
          buffer = self.client.recv(timeout_ms=1200)
        else:
          while True:
            b = self.client.recv(timeout_ms=0)
            if b is None:
              break
            buffer = b
        if buffer is None:
          self._send_status()
          time.sleep(0.02)
          continue

        self.frames_received += 1
        self.last_frame_mono_ns = time.monotonic_ns()
        self.last_frame_id = int(getattr(self.client, 'frame_id', -1) or -1)
        self.camera_width = int(self.client.width); self.camera_height = int(self.client.height); self.camera_stride = int(self.client.stride)

        if self.snapshot_request.is_set():
          exposure_ready = self.onroad or not self._snapshot_driver_view_owned or (now - self._snapshot_driver_view_started_at >= 1.2)
          if exposure_ready:
            self._capture_snapshot(buffer)

        if not (self.onroad and enabled and self.config_loaded and self.inference.valid):
          self._send_status(force=True)
          time.sleep(0.03)
          continue

        raw = np.frombuffer(buffer.data, dtype=np.uint8).reshape((len(buffer.data) // self.client.stride, self.client.stride))
        if self.client.stride != self.client.width:
          raw = raw[:, :self.client.width]
        uv_offset = int(getattr(buffer, 'uv_offset', 0) or 0)
        y_plane_rows = (uv_offset // int(self.client.stride)) if uv_offset > 0 else _nv12_y_rows(self.client.width, self.client.height, self.client.stride)

        interval = self._inference_interval(now)
        if self.last_inference_at and now - self.last_inference_at < interval - 0.015:
          self._send_status()
          time.sleep(0.02)
          continue

        sides = self.inference.configured_sides
        if not sides:
          self.config_loaded = False
          self._send_status()
          time.sleep(0.05)
          continue
        if self.current_side not in sides:
          self.current_side = sides[0]

        last_side = self.last_inference_at_side[self.current_side]
        dt = (now - last_side) if last_side else interval
        conf, smooth, hold_off = self._config_values()
        t0 = time.perf_counter_ns()
        l_active, r_active = self.inference.update(raw, self.client.width, self.client.height, dt, conf, smooth, self.current_side, hold_off, y_plane_rows=y_plane_rows)
        self.inference_ms = (time.perf_counter_ns() - t0) / 1e6
        self.last_inference_at = now
        self.last_inference_at_side[self.current_side] = now
        self.last_inference_mono_ns = time.monotonic_ns()
        if l_active or r_active:
          self.followup_until = now + FOLLOWUP_WINDOW
        idx = sides.index(self.current_side)
        self.current_side = sides[(idx + 1) % len(sides)]
        self._send_status(force=True)

        if now - self.last_status_log >= 10.0:
          self.last_status_log = now
          s = self.status()
          print(f"[G80 SIDE VISION] model={s['model_valid']} cfg={s['config_loaded']} cam={s['camera_connected']} L={s['left']['score']:.3f} R={s['right']['score']:.3f} inf={s['inference_ms']:.1f}ms throttle={s['throttle_factor']:.2f}x", flush=True)
      except Exception as e:
        self.last_error = f'run: {e!r}'
        self._send_status(force=True)
        time.sleep(0.2)

      # Bound daemon loop overhead independent of radar process.
      elapsed = time.monotonic() - loop_t0
      if elapsed < 0.02:
        time.sleep(0.02 - elapsed)


def main():
  # Sunnypilot's PythonProcess wrapper has no nice= argument on this branch.
  # Lower this experimental vision daemon's scheduler priority in-process.
  try:
    os.nice(10)
  except Exception:
    pass
  SideVisionDaemon().run()


if __name__ == '__main__':
  main()
