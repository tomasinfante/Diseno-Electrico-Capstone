#!/usr/bin/env python3
"""
Interfaz de seguimiento - Raspberry Pi 4 + Camera Module 2
Dashboard web con:
    1. Video en tiempo real con el objetivo detectado
    2. Error de posición del objetivo respecto al centro de la imagen (px y grados)
    3. Definición (resolución) de la imagen que se está recibiendo
    4. FPS del video

Instalación (Raspberry Pi OS Bookworm):
    sudo apt install -y python3-picamera2 python3-opencv python3-flask

Uso:
    python3 interfaz.py
    Abrir en el navegador: http://<ip-de-la-raspberry>:5000

Si picamera2 no está instalado (p. ej. en un PC), usa la primera webcam vía OpenCV.
"""

import threading
import time
from collections import deque

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

# ---------------- Parámetros ----------------
ANCHO, ALTO = 640, 480
FPS_CAMARA = 30
JPEG_CALIDAD = 70
PUERTO = 5000

# Modo del sensor (raw) a forzar. None = lo elige libcamera, que para 640x480
# puede usar un modo recortado y reducir el campo de visión.
# (1640, 1232) = campo de visión completo con binning 2x2.
MODO_SENSOR = None

# Camera Module 2 (IMX219), sensor completo: campo de visión horizontal y vertical [°]
FOV_H, FOV_V = 62.2, 48.8

ROJO_BAJO_1 = np.array([0, 120, 70])
ROJO_ALTO_1 = np.array([10, 255, 255])
ROJO_BAJO_2 = np.array([170, 120, 70])
ROJO_ALTO_2 = np.array([179, 255, 255])

AREA_MIN = 70           # px^2, descarta ruido
CIRCULARIDAD_MIN = 0.6  # 1.0 = círculo perfecto

HISTORIAL_MAX = 600     # muestras guardadas para los gráficos (~20 s a 30 FPS)

# ---------------- Estado compartido ----------------
lock = threading.Lock()
nuevo_frame = threading.Condition(lock)
ultimo_jpg = None
id_frame = 0
estado = None
historial = deque(maxlen=HISTORIAL_MAX)  # [id, t, detectado, ex, ey, fps]


# ---------------- Cámaras ----------------
def fov_recortado(frac_h, frac_v):
    """Campo de visión [°] de una ventana que ocupa frac_h x frac_v del sensor."""
    return (2 * np.degrees(np.arctan(frac_h * np.tan(np.radians(FOV_H / 2)))),
            2 * np.degrees(np.arctan(frac_v * np.tan(np.radians(FOV_V / 2)))))


class CamaraPi:
    """Camera Module 2 vía Picamera2."""

    def __init__(self):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        extra = {"raw": {"size": MODO_SENSOR}} if MODO_SENSOR else {}
        # En Picamera2, "RGB888" entrega los píxeles en orden BGR (el que usa OpenCV)
        config = self.picam2.create_video_configuration(
            main={"size": (ANCHO, ALTO), "format": "RGB888"},
            controls={"FrameRate": FPS_CAMARA},
            **extra,
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)

        props = self.picam2.camera_properties
        tamano = tuple(props.get("PixelArraySize", (0, 0)))
        areas = props.get("PixelArrayActiveAreas") or [(0, 0, *tamano)]
        self.activo = tuple(areas[0][2:4])  # zona útil del sensor (referencia del FOV)

        cfg = self.picam2.camera_configuration()
        modo = (cfg.get("sensor") or {}).get("output_size") or (cfg.get("raw") or {}).get("size")

        self.info = {
            "fuente": f"Picamera2 · {props.get('Model', '?')}",
            "sensor": list(self.activo),
            "modo_sensor": list(modo) if modo else None,
            "fov_supuesto": False,
        }

    def leer(self):
        req = self.picam2.capture_request()
        try:
            frame = req.make_array("main")
            meta = req.get_metadata()
        finally:
            req.release()

        duracion = meta.get("FrameDuration")  # µs
        recorte = meta.get("ScalerCrop")       # (x, y, ancho, alto) en píxeles del sensor
        if recorte:
            fov = fov_recortado(recorte[2] / self.activo[0], recorte[3] / self.activo[1])
        else:
            fov = (FOV_H, FOV_V)
        return frame, {
            "fps_nominal": 1e6 / duracion if duracion else None,
            "recorte": list(recorte[2:4]) if recorte else None,
            "fov": fov,
        }


class CamaraWebcam:
    """Webcam vía OpenCV, para probar la interfaz en un PC sin picamera2."""

    def __init__(self, indice=0):
        self.cap = cv2.VideoCapture(indice)
        if not self.cap.isOpened():
            raise RuntimeError("No se encontró ninguna cámara")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO)
        self.cap.set(cv2.CAP_PROP_FPS, FPS_CAMARA)
        self.info = {"fuente": "Webcam (OpenCV)", "sensor": None,
                     "modo_sensor": None, "fov_supuesto": True}

    def leer(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("No se pudo leer la webcam")
        return frame, {"fps_nominal": self.cap.get(cv2.CAP_PROP_FPS) or None,
                       "recorte": None, "fov": (FOV_H, FOV_V)}


def abrir_camara():
    try:
        import picamera2  # noqa: F401
    except ImportError:
        print("picamera2 no disponible: usando webcam")
        return CamaraWebcam()
    return CamaraPi()


# ---------------- Visión ----------------
def detectar(frame_bgr):
    """Devuelve (x, y, radio) de la mejor pelota roja, o None."""
    blur = cv2.GaussianBlur(frame_bgr, (7, 7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    mascara = cv2.inRange(hsv, ROJO_BAJO_1, ROJO_ALTO_1) | \
              cv2.inRange(hsv, ROJO_BAJO_2, ROJO_ALTO_2)

    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel, iterations=1)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel, iterations=2)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor, mejor_area = None, 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area < AREA_MIN:
            continue
        perimetro = cv2.arcLength(c, True)
        if perimetro == 0:
            continue
        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad < CIRCULARIDAD_MIN:
            continue
        if area > mejor_area:
            (x, y), r = cv2.minEnclosingCircle(c)
            mejor, mejor_area = (float(x), float(y), float(r)), area

    return mejor


def calcular_error(x, y, ancho, alto, fov):
    """
    Error del objetivo respecto al centro de la imagen, en px y en grados.
    Convención (igual que lqr_pan_tilt.py): ex + hacia la derecha, ey + hacia abajo.
    """
    ex = x - ancho / 2
    ey = y - alto / 2
    f_x = (ancho / 2) / np.tan(np.radians(fov[0] / 2))  # focal en píxeles
    f_y = (alto / 2) / np.tan(np.radians(fov[1] / 2))
    return ex, ey, float(np.degrees(np.arctan(ex / f_x))), float(np.degrees(np.arctan(ey / f_y)))


FUENTE = cv2.FONT_HERSHEY_SIMPLEX


def etiqueta(frame, texto, x, y, escala, grosor):
    """Texto blanco sobre un fondo oscurecido. (x, y) = esquina superior izquierda."""
    (tw, th), base = cv2.getTextSize(texto, FUENTE, escala, grosor)
    m = 3 * grosor
    alto, ancho = frame.shape[:2]
    x = int(min(max(x, 0), ancho - tw - 2 * m))
    y = int(min(max(y, 0), alto - th - base - 2 * m))
    frame[y:y + th + base + 2 * m, x:x + tw + 2 * m] //= 2
    cv2.putText(frame, texto, (x + m, y + m + th), FUENTE, escala, (255, 255, 255), grosor, cv2.LINE_AA)


def dibujar(frame, objetivo, error, fps):
    alto, ancho = frame.shape[:2]
    cx, cy = ancho // 2, alto // 2
    g = max(1, round(ancho / 640))  # grosor proporcional a la resolución

    cv2.drawMarker(frame, (cx, cy), (220, 220, 220), cv2.MARKER_CROSS, 24 * g, g)

    if objetivo:
        x, y, r = (int(round(v)) for v in objetivo)
        largo = max(np.hypot(x - cx, y - cy), 1)
        cv2.arrowedLine(frame, (cx, cy), (x, y), (0, 220, 255), g, cv2.LINE_AA,
                        tipLength=min(0.3, 12 * g / largo))
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2 * g, cv2.LINE_AA)
        cv2.circle(frame, (x, y), 3 * g, (0, 255, 0), -1)

        texto = f"({error[0]:+.0f}, {error[1]:+.0f}) px"
        tw = cv2.getTextSize(texto, FUENTE, 0.5 * g, g)[0][0] + 6 * g
        tx = x + r + 6 if x + r + 6 + tw < ancho else x - r - 6 - tw
        etiqueta(frame, texto, tx, y - 10 * g, 0.5 * g, g)

    etiqueta(frame, f"{ancho}x{alto}  {fps:.1f} FPS", 6, 6, 0.55 * g, g)


def bucle_camara():
    global ultimo_jpg, id_frame, estado

    camara = abrir_camara()
    t0 = time.time()
    llegadas = deque()  # instantes de llegada del último segundo

    while True:
        frame, meta = camara.leer()
        t = time.time()
        t_ini = time.perf_counter()

        llegadas.append(t)
        while t - llegadas[0] > 1.0:
            llegadas.popleft()
        fps = (len(llegadas) - 1) / (t - llegadas[0]) if len(llegadas) > 1 else 0.0

        frame = np.ascontiguousarray(frame)
        alto, ancho = frame.shape[:2]

        objetivo = detectar(frame)
        error = calcular_error(objetivo[0], objetivo[1], ancho, alto, meta["fov"]) if objetivo else None

        dibujar(frame, objetivo, error, fps)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_CALIDAD])
        if not ok:
            continue
        proc_ms = (time.perf_counter() - t_ini) * 1000

        ex, ey = (round(error[0], 1), round(error[1], 1)) if error else (None, None)
        nuevo_estado = {
            **camara.info,
            "detectado": objetivo is not None,
            "x": round(objetivo[0], 1) if objetivo else None,
            "y": round(objetivo[1], 1) if objetivo else None,
            "radio": round(objetivo[2], 1) if objetivo else None,
            "ex": ex,
            "ey": ey,
            "ex_deg": round(error[2], 2) if error else None,
            "ey_deg": round(error[3], 2) if error else None,
            "e_mag": round(float(np.hypot(error[0], error[1])), 1) if error else None,
            "fps": round(fps, 1),
            "fps_nominal": round(meta["fps_nominal"], 1) if meta["fps_nominal"] else None,
            "proc_ms": round(proc_ms, 1),
            "ancho": ancho,
            "alto": alto,
            "recorte": meta["recorte"],
            "fov": [round(float(v), 1) for v in meta["fov"]],
            "jpg_kb": round(len(jpg) / 1024, 1),
            "jpeg_calidad": JPEG_CALIDAD,
        }

        with lock:
            id_frame += 1
            ultimo_jpg = jpg.tobytes()
            estado = nuevo_estado
            historial.append([id_frame, round(t - t0, 3), objetivo is not None, ex, ey, round(fps, 1)])
            nuevo_frame.notify_all()


# ---------------- Servidor web ----------------
app = Flask(__name__)

PAGINA = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Interfaz de seguimiento</title>
<style>
:root{
  color-scheme:dark;
  --bg:#0f0f0e; --surface:#1a1a19; --border:#2e2e2b; --grid:#2a2a28;
  --text:#f0efec; --text-2:#c3c2b7; --muted:#8b8a82;
  --s1:#3987e5; --s2:#d95926;
  --good:#0ca30c; --critical:#d03b3b;
}
@media (prefers-color-scheme: light){
  :root{
    color-scheme:light;
    --bg:#f3f3f1; --surface:#fcfcfb; --border:#dddcd7; --grid:#e8e7e3;
    --text:#0b0b0b; --text-2:#52514e; --muted:#7a7974;
    --s1:#2a78d6; --s2:#eb6834;
  }
}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--text);
     font:14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif}
header{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:16px}
h1{font-size:18px;font-weight:600;margin:0 auto 0 0}
.pill{display:inline-flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;
      border:1px solid var(--border);background:var(--surface);font-size:13px;white-space:nowrap}
.dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}
.ico{font-weight:700}
.ok{color:var(--good)} .no{color:var(--critical)}
.dot.ok{background:var(--good)} .dot.no{background:var(--critical)}
.grid{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(300px,1fr);gap:16px;align-items:start}
.col{display:flex;flex-direction:column;gap:16px;min-width:0}
@media (max-width:900px){.grid{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:14px 16px;min-width:0}
.card h2{display:flex;justify-content:space-between;align-items:baseline;gap:8px;margin:0 0 12px;
         font-size:12px;font-weight:600;letter-spacing:.05em;text-transform:uppercase;color:var(--text-2)}
.card h2 span{font-weight:400;letter-spacing:0;text-transform:none;color:var(--muted)}
#video{display:block;width:100%;aspect-ratio:4/3;object-fit:contain;background:#000;border-radius:6px}
.num{font-variant-numeric:tabular-nums}
.kpis{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-bottom:12px}
.kpi .lbl{display:block;font-size:12px;color:var(--muted)}
.kpi b{display:block;font-size:20px;font-weight:600;font-variant-numeric:tabular-nums;white-space:nowrap}
.kpi small{color:var(--text-2);font-variant-numeric:tabular-nums}
.sw{display:inline-block;width:12px;height:3px;border-radius:2px;vertical-align:middle;margin-right:6px}
#mira{display:block;width:100%;aspect-ratio:4/3;max-height:280px}
.nota{margin:8px 0 0;font-size:12px;color:var(--muted)}
.hero{display:flex;align-items:baseline;gap:6px}
.hero b{font-size:40px;font-weight:600;line-height:1.1;font-variant-numeric:tabular-nums}
.hero span{color:var(--muted)}
.sub{color:var(--text-2);font-size:13px;margin:4px 0 10px}
.sub b{font-weight:600;color:var(--text);font-variant-numeric:tabular-nums}
.tabla{display:grid;grid-template-columns:auto 1fr;gap:6px 16px;margin:0}
.tabla dt{color:var(--muted)}
.tabla dd{margin:0;font-variant-numeric:tabular-nums}
.leyenda{display:flex;gap:16px;font-size:12px;color:var(--text-2);margin-bottom:6px}
.chart-wrap{position:relative}
.chart{display:block;width:100%;height:220px;touch-action:none}
.chart.small{height:110px}
.tip{position:absolute;top:4px;pointer-events:none;background:var(--surface);border:1px solid var(--border);
     border-radius:6px;padding:6px 8px;font-size:12px;white-space:nowrap;
     box-shadow:0 4px 12px rgba(0,0,0,.25);font-variant-numeric:tabular-nums}
.tip .t{color:var(--muted);margin-bottom:2px}
</style></head><body>
<header>
  <h1>Interfaz de seguimiento</h1>
  <span class="pill" id="estado"><span class="ico">…</span> Esperando datos</span>
  <span class="pill"><span class="dot" id="conn-dot"></span><span id="conn">Conectando</span></span>
</header>

<div class="grid">
  <div class="col">
    <section class="card">
      <h2>Video en tiempo real <span class="num" id="tag-res">—</span></h2>
      <img id="video" src="/video" alt="Video de la cámara con el objetivo detectado">
      <p class="nota">Cruz = centro de la imagen · círculo verde = objetivo · flecha = error de posición</p>
    </section>
    <section class="card">
      <h2>Error de posición vs tiempo <span>últimos 10 s</span></h2>
      <div class="leyenda">
        <span><i class="sw" style="background:var(--s1)"></i>e<sub>x</sub> horizontal</span>
        <span><i class="sw" style="background:var(--s2)"></i>e<sub>y</sub> vertical</span>
      </div>
      <div class="chart-wrap"><canvas id="g-error" class="chart"></canvas><div class="tip" hidden></div></div>
    </section>
  </div>

  <div class="col">
    <section class="card">
      <h2>Error de posición <span>respecto al centro</span></h2>
      <div class="kpis">
        <div class="kpi"><span class="lbl"><i class="sw" style="background:var(--s1)"></i>e<sub>x</sub></span>
          <b id="ex">—</b><small id="ex-deg">—</small></div>
        <div class="kpi"><span class="lbl"><i class="sw" style="background:var(--s2)"></i>e<sub>y</sub></span>
          <b id="ey">—</b><small id="ey-deg">—</small></div>
        <div class="kpi"><span class="lbl">|e|</span>
          <b id="em">—</b><small id="pos">—</small></div>
      </div>
      <canvas id="mira"></canvas>
      <p class="nota">x+ derecha, y+ abajo · grados según el campo de visión efectivo</p>
    </section>

    <section class="card">
      <h2>FPS del video</h2>
      <div class="hero"><b id="fps">—</b><span>FPS medidos</span></div>
      <div class="sub">Nominal cámara <b id="fps-nom">—</b> · Procesamiento <b id="proc">—</b> ms/frame</div>
      <div class="chart-wrap"><canvas id="g-fps" class="chart small"></canvas><div class="tip" hidden></div></div>
    </section>

    <section class="card">
      <h2>Definición de la imagen</h2>
      <dl class="tabla">
        <dt>Resolución recibida</dt><dd id="res">—</dd>
        <dt>Píxeles</dt><dd id="mp">—</dd>
        <dt>Sensor</dt><dd id="sensor">—</dd>
        <dt>Modo del sensor</dt><dd id="modo">—</dd>
        <dt>Recorte del sensor</dt><dd id="recorte">—</dd>
        <dt>Campo de visión</dt><dd id="fov">—</dd>
        <dt>JPEG enviado</dt><dd id="jpg">—</dd>
        <dt>Fuente</dt><dd id="fuente">—</dd>
      </dl>
    </section>
  </div>
</div>

<script>
const $ = id => document.getElementById(id);
const VENTANA = 10;   // s mostrados en los gráficos
const ESTELA = 1.0;   // s de estela en la mira
let muestras = [];    // [id, t, detectado, ex, ey, fps]
let ultimoSeq = -1;
let actual = null;
let conectado = null;
let aspectoActual = '';

const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const firmado = (v, d = 1) => (v < 0 ? '−' : '+') + Math.abs(v).toFixed(d);
const par = a => a ? `${a[0]} × ${a[1]}` : '—';

function techoBonito(v) {   // 1, 2, 5 × 10^k
  if (!(v > 0)) return 1;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  for (const m of [1, 2, 5, 10]) if (m * p >= v) return m * p;
}

function aspecto(w, h) {
  const mcd = (a, b) => b ? mcd(b, a % b) : a;
  const g = mcd(w, h);
  return (w / g <= 32) ? `${w / g}:${h / g}` : `${(w / h).toFixed(2)}:1`;
}

function prepararCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return {ctx, w, h};
}

// ---------- Gráfico de líneas con tooltip ----------
class GraficoLinea {
  constructor(id, cfg) {
    this.cv = $(id); this.cfg = cfg; this.hx = null;
    this.tip = this.cv.parentElement.querySelector('.tip');
    this.cv.addEventListener('pointermove', e => { this.hx = e.offsetX; this.dibujar(); });
    this.cv.addEventListener('pointerleave', () => { this.hx = null; this.dibujar(); });
  }

  dibujar() {
    const {ctx, w, h} = prepararCanvas(this.cv);
    const c = this.cfg;
    const M = {l: 44, r: c.etiquetas ? 28 : 10, t: 8, b: 20};
    const pw = w - M.l - M.r, ph = h - M.t - M.b;
    if (pw <= 0 || ph <= 0) return;

    const tFin = muestras.length ? muestras[muestras.length - 1][1] : 0;
    const tIni = tFin - VENTANA;
    const [y0, y1, ticks] = c.rango();
    const X = t => M.l + (t - tIni) / VENTANA * pw;
    const Y = v => M.t + (1 - (v - y0) / (y1 - y0)) * ph;
    const col = {grid: css('--grid'), muted: css('--muted'), text2: css('--text-2'), surf: css('--surface')};

    // Grilla y eje y
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const v of ticks) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = (v === 0 && y0 < 0) ? col.muted : col.grid;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(M.l, y); ctx.lineTo(M.l + pw, y); ctx.stroke();
      ctx.fillStyle = col.muted; ctx.fillText(c.fmtTick(v), M.l - 6, y);
    }
    // Eje x
    ctx.textBaseline = 'top';
    [[VENTANA, 'left', `−${VENTANA} s`], [VENTANA / 2, 'center', `−${VENTANA / 2} s`], [0, 'right', 'ahora']]
      .forEach(([s, a, txt]) => { ctx.textAlign = a; ctx.fillText(txt, X(tFin - s), M.t + ph + 6); });

    // Líneas de referencia
    for (const r of (c.referencias ? c.referencias() : [])) {
      const y = Math.round(Y(r.v)) + 0.5;
      ctx.strokeStyle = col.muted; ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(M.l, y); ctx.lineTo(M.l + pw, y); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = col.muted; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
      ctx.fillText(r.etiqueta, M.l + 4, y - 2);
    }

    // Series (se cortan donde no hubo detección)
    ctx.save();
    ctx.beginPath(); ctx.rect(M.l, M.t, pw, ph); ctx.clip();
    ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    for (const s of c.series) {
      ctx.strokeStyle = css(s.color);
      ctx.beginPath();
      let abierto = false;
      for (const m of muestras) {
        const v = m[s.i];
        if (v === null) { abierto = false; continue; }
        if (abierto) ctx.lineTo(X(m[1]), Y(v)); else { ctx.moveTo(X(m[1]), Y(v)); abierto = true; }
      }
      ctx.stroke();
    }
    ctx.restore();

    // Etiquetas directas al final de cada serie
    if (c.etiquetas && muestras.length) {
      const ult = muestras[muestras.length - 1];
      const etq = c.series.filter(s => ult[s.i] !== null).map(s => ({s, y: Y(ult[s.i])}));
      if (etq.length === 2 && Math.abs(etq[0].y - etq[1].y) < 12) {
        const mid = (etq[0].y + etq[1].y) / 2, arriba = etq[0].y <= etq[1].y ? 0 : 1;
        etq[arriba].y = mid - 6; etq[1 - arriba].y = mid + 6;
      }
      ctx.fillStyle = col.text2; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      for (const e of etq) ctx.fillText(e.s.nombre, M.l + pw + 6, e.y);
    }

    // Hover: cursor vertical + tooltip
    if (this.hx === null || !muestras.length || this.hx < M.l || this.hx > M.l + pw) {
      this.tip.hidden = true; return;
    }
    const tH = tIni + (this.hx - M.l) / pw * VENTANA;
    let m = muestras[0];
    for (const q of muestras) if (Math.abs(q[1] - tH) < Math.abs(m[1] - tH)) m = q;
    const px = X(m[1]);
    ctx.strokeStyle = col.muted; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(Math.round(px) + 0.5, M.t); ctx.lineTo(Math.round(px) + 0.5, M.t + ph); ctx.stroke();

    let html = `<div class="t">${(m[1] - tFin).toFixed(1)} s</div>`;
    for (const s of c.series) {
      const v = m[s.i];
      if (v === null) continue;
      ctx.beginPath(); ctx.arc(px, Y(v), 4, 0, 2 * Math.PI);
      ctx.fillStyle = css(s.color); ctx.fill();
      ctx.strokeStyle = col.surf; ctx.lineWidth = 2; ctx.stroke();
      html += `<div><i class="sw" style="background:var(${s.color})"></i>${s.nombre} <b>${c.fmt(v)}</b></div>`;
    }
    if (c.series.every(s => m[s.i] === null)) html += '<div>Sin objetivo</div>';
    this.tip.innerHTML = html;
    this.tip.hidden = false;
    const tw = this.tip.offsetWidth;
    this.tip.style.left = (px + 12 + tw > w ? px - 12 - tw : px + 12) + 'px';
  }
}

const gError = new GraficoLinea('g-error', {
  series: [{i: 3, nombre: 'ex', color: '--s1'}, {i: 4, nombre: 'ey', color: '--s2'}],
  etiquetas: true,
  rango() {
    let m = 10;
    for (const q of muestras) if (q[2]) m = Math.max(m, Math.abs(q[3]), Math.abs(q[4]));
    const t = techoBonito(m);
    return [-t, t, [-t, -t / 2, 0, t / 2, t]];
  },
  fmtTick: v => v === 0 ? '0' : firmado(v, Number.isInteger(v) ? 0 : 1),
  fmt: v => firmado(v) + ' px',
});

const gFps = new GraficoLinea('g-fps', {
  series: [{i: 5, nombre: 'FPS', color: '--s1'}],
  rango() {
    let m = (actual && actual.fps_nominal) || 0;
    for (const q of muestras) m = Math.max(m, q[5]);
    const t = techoBonito(Math.max(m * 1.1, 5));
    return [0, t, [0, t / 2, t]];
  },
  referencias: () => (actual && actual.fps_nominal) ? [{v: actual.fps_nominal, etiqueta: 'nominal'}] : [],
  fmtTick: v => String(v),
  fmt: v => v.toFixed(1) + ' FPS',
});

// ---------- Mira: posición del objetivo en el cuadro ----------
function dibujarMira() {
  const {ctx, w, h} = prepararCanvas($('mira'));
  if (!actual) return;
  const W = actual.ancho, H = actual.alto;
  const k = Math.min(w / W, h / H);
  const ox = (w - W * k) / 2, oy = (h - H * k) / 2;
  const P = (ex, ey) => [ox + (ex + W / 2) * k, oy + (ey + H / 2) * k];
  const [cx, cy] = P(0, 0);
  const col = {grid: css('--grid'), border: css('--border'), muted: css('--muted'),
               text: css('--text'), text2: css('--text-2'), surf: css('--surface')};

  ctx.strokeStyle = col.border; ctx.lineWidth = 1;
  ctx.strokeRect(ox + 0.5, oy + 0.5, W * k - 1, H * k - 1);
  ctx.strokeStyle = col.grid;
  for (const f of [0.25, 0.5, 0.75]) {
    ctx.beginPath(); ctx.arc(cx, cy, f * H / 2 * k, 0, 2 * Math.PI); ctx.stroke();
  }
  ctx.strokeStyle = col.muted;
  ctx.beginPath();
  ctx.moveTo(ox, Math.round(cy) + 0.5); ctx.lineTo(ox + W * k, Math.round(cy) + 0.5);
  ctx.moveTo(Math.round(cx) + 0.5, oy); ctx.lineTo(Math.round(cx) + 0.5, oy + H * k);
  ctx.stroke();

  // Estela del último segundo
  const tFin = muestras.length ? muestras[muestras.length - 1][1] : 0;
  ctx.fillStyle = col.text2;
  for (const q of muestras) {
    if (!q[2] || q[1] < tFin - ESTELA) continue;
    const [x, y] = P(q[3], q[4]);
    ctx.globalAlpha = 0.5 * (1 - (tFin - q[1]) / ESTELA);
    ctx.beginPath(); ctx.arc(x, y, 2.5, 0, 2 * Math.PI); ctx.fill();
  }
  ctx.globalAlpha = 1;

  if (!actual.detectado) return;
  const [px, py] = P(actual.ex, actual.ey);
  ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
  ctx.strokeStyle = css('--s1');
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(px, cy); ctx.stroke();
  ctx.strokeStyle = css('--s2');
  ctx.beginPath(); ctx.moveTo(px, cy); ctx.lineTo(px, py); ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle = col.text; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(px, py); ctx.stroke();
  ctx.beginPath(); ctx.arc(px, py, 5, 0, 2 * Math.PI);
  ctx.fillStyle = col.text; ctx.fill();
  ctx.strokeStyle = col.surf; ctx.lineWidth = 2; ctx.stroke();
}

// ---------- Paneles numéricos ----------
function pintarEstado(d) {
  const det = d.detectado;
  $('estado').innerHTML = det
    ? '<span class="ico ok">✓</span> Objetivo detectado'
    : '<span class="ico no">✕</span> Sin objetivo';

  $('ex').textContent = det ? firmado(d.ex) + ' px' : '—';
  $('ey').textContent = det ? firmado(d.ey) + ' px' : '—';
  $('em').textContent = det ? d.e_mag.toFixed(1) + ' px' : '—';
  $('ex-deg').textContent = det ? firmado(d.ex_deg, 2) + '°' : '—';
  $('ey-deg').textContent = det ? firmado(d.ey_deg, 2) + '°' : '—';
  $('pos').textContent = det ? `(${d.x.toFixed(0)}, ${d.y.toFixed(0)}) r=${d.radio.toFixed(0)}` : '—';

  $('fps').textContent = d.fps.toFixed(1);
  $('fps-nom').textContent = d.fps_nominal ? d.fps_nominal.toFixed(1) : '—';
  $('proc').textContent = d.proc_ms.toFixed(1);

  $('tag-res').textContent = `${d.ancho}×${d.alto}`;
  $('res').textContent = `${d.ancho} × ${d.alto} px`;
  $('mp').textContent = `${(d.ancho * d.alto / 1e6).toFixed(2)} MP · ${aspecto(d.ancho, d.alto)}`;
  $('sensor').textContent = par(d.sensor);
  $('modo').textContent = par(d.modo_sensor);
  $('recorte').textContent = d.recorte && d.sensor
    ? `${par(d.recorte)} (${(100 * d.recorte[0] / d.sensor[0]).toFixed(0)} % del ancho)` : '—';
  $('fov').textContent = `${d.fov_supuesto ? '≈ ' : ''}${d.fov[0].toFixed(1)}° × ${d.fov[1].toFixed(1)}°`
    + (d.fov_supuesto ? ' (supuesto)' : '');
  $('jpg').textContent = `${d.jpg_kb.toFixed(1)} kB/frame · calidad ${d.jpeg_calidad} · ≈ ${(d.jpg_kb * d.fps / 1024).toFixed(2)} MB/s`;
  $('fuente').textContent = d.fuente;

  const ar = `${d.ancho} / ${d.alto}`;
  if (ar !== aspectoActual) {
    aspectoActual = ar;
    $('video').style.aspectRatio = ar;
    $('mira').style.aspectRatio = ar;
  }
}

function marcarConexion(ok) {
  if (ok === conectado) return;
  if (ok && conectado === false) $('video').src = '/video?' + Date.now();  // reabrir el stream
  conectado = ok;
  $('conn').textContent = ok ? 'En vivo' : 'Sin conexión';
  $('conn-dot').className = 'dot ' + (ok ? 'ok' : 'no');
}

function dibujarTodo() { gError.dibujar(); gFps.dibujar(); dibujarMira(); }

async function actualizar() {
  try {
    const r = await fetch('/datos?desde=' + ultimoSeq, {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    const d = await r.json();
    if (d.seq < ultimoSeq) muestras = [];   // el servidor se reinició
    muestras.push(...d.muestras);
    ultimoSeq = d.seq;
    if (muestras.length) {
      const tFin = muestras[muestras.length - 1][1];
      const i = muestras.findIndex(m => m[1] >= tFin - VENTANA);
      if (i > 0) muestras.splice(0, i);
    }
    if (d.actual) { actual = d.actual; pintarEstado(actual); }
    dibujarTodo();
    marcarConexion(true);
  } catch (e) {
    marcarConexion(false);
  }
  setTimeout(actualizar, 100);
}

new ResizeObserver(dibujarTodo).observe(document.body);
matchMedia('(prefers-color-scheme: light)').addEventListener('change', dibujarTodo);
actualizar();
</script></body></html>"""


@app.route("/")
def index():
    return PAGINA


@app.route("/datos")
def datos():
    desde = request.args.get("desde", -1, type=int)
    with lock:
        if desde > id_frame:  # el servidor se reinició: enviar todo el historial
            desde = -1
        muestras = [m for m in historial if m[0] > desde]
        return jsonify(seq=id_frame, actual=estado, muestras=muestras)


def generador_mjpeg():
    visto = -1
    while True:
        with nuevo_frame:
            nuevo_frame.wait_for(lambda: id_frame != visto, timeout=1.0)
            jpg, visto = ultimo_jpg, id_frame
        if jpg is not None:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"


@app.route("/video")
def video():
    return Response(generador_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    threading.Thread(target=bucle_camara, daemon=True).start()
    app.run(host="0.0.0.0", port=PUERTO, threaded=True)
