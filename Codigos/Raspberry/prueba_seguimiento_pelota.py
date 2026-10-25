#!/usr/bin/env python3
"""
Seguimiento de pelota de golf roja - Raspberry Pi 4 + Camera Module 2
Captura continua en RAM -> detección HSV -> dashboard web (MJPEG + datos JSON)

Instalación (Raspberry Pi OS Bookworm):
    sudo apt install -y python3-picamera2 python3-opencv python3-flask

Uso:
    python3 seguimiento_pelota.py
    Abrir en el navegador: http://<ip-de-la-raspberry>:5000
"""

import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify
from picamera2 import Picamera2

# ---------------- Parámetros ----------------
ANCHO, ALTO = 640, 480
JPEG_CALIDAD = 70

# Modo del sensor: campo de visión completo con binning 2x2, escalado a ANCHO x ALTO.
# Sin forzarlo, libcamera puede elegir el modo nativo 640x480 (recorte, FOV ~26.5°).
MODO_SENSOR = (1640, 1232)

ROJO_BAJO_1 = np.array([0, 120, 70])
ROJO_ALTO_1 = np.array([10, 255, 255])
ROJO_BAJO_2 = np.array([170, 120, 70])
ROJO_ALTO_2 = np.array([179, 255, 255])

AREA_MIN = 70          # px^2, descarta ruido
CIRCULARIDAD_MIN = 0.6  # 1.0 = círculo perfecto

# ---------------- Estado compartido ----------------
lock = threading.Lock()
ultimo_jpg = None
ultimo_dato = {"detectado": False, "x": None, "y": None, "radio": None, "fps": 0.0}


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
            mejor, mejor_area = (int(x), int(y), int(r)), area

    return mejor, mascara


def bucle_camara():
    global ultimo_jpg, ultimo_dato

    picam2 = Picamera2()
    # En Picamera2, "RGB888" entrega los píxeles en orden BGR (el que usa OpenCV)
    config = picam2.create_video_configuration(
        main={"size": (ANCHO, ALTO), "format": "RGB888"},
        raw={"size": MODO_SENSOR},
        controls={"FrameRate": 30},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)  

    t_prev = time.time()
    fps = 0.0

    while True:
        frame = picam2.capture_array()
        resultado, _ = detectar(frame)

        if resultado:
            x, y, r = resultado
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
            cv2.putText(frame, f"({x},{y})", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        t = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(t - t_prev, 1e-6))
        t_prev = t
        cv2.putText(frame, f"{fps:.1f} FPS", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_CALIDAD])
        if not ok:
            continue

        with lock:
            ultimo_jpg = jpg.tobytes()
            ultimo_dato = {
                "detectado": resultado is not None,
                "x": resultado[0] if resultado else None,
                "y": resultado[1] if resultado else None,
                "radio": resultado[2] if resultado else None,
                "fps": round(fps, 1),
            }


# ---------------- Servidor web ----------------
app = Flask(__name__)

PAGINA = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Seguimiento pelota</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
 .wrap{display:flex;gap:16px;flex-wrap:wrap}
 img{max-width:100%;border:1px solid #333;border-radius:6px}
 .panel{min-width:220px;background:#1c1c1c;padding:12px 16px;border-radius:6px}
 .ok{color:#4ade80}.no{color:#f87171}
 td{padding:4px 8px}
</style></head><body>
<h2>Seguimiento de pelota roja</h2>
<div class="wrap">
  <img src="/video" alt="cámara">
  <div class="panel">
    <p id="estado">—</p>
    <table>
      <tr><td>x</td><td id="x">—</td></tr>
      <tr><td>y</td><td id="y">—</td></tr>
      <tr><td>radio</td><td id="r">—</td></tr>
      <tr><td>FPS</td><td id="fps">—</td></tr>
    </table>
  </div>
</div>
<script>
async function actualizar(){
  try{
    const d = await (await fetch('/datos')).json();
    const e = document.getElementById('estado');
    e.textContent = d.detectado ? 'Objetivo detectado' : 'Sin objetivo';
    e.className = d.detectado ? 'ok' : 'no';
    x.textContent = d.x ?? '—'; y.textContent = d.y ?? '—';
    r.textContent = d.radio ?? '—'; fps.textContent = d.fps;
  }catch(err){}
}
setInterval(actualizar, 200);
</script></body></html>"""


@app.route("/")
def index():
    return PAGINA


@app.route("/datos")
def datos():
    with lock:
        return jsonify(ultimo_dato)


def generador_mjpeg():
    while True:
        with lock:
            jpg = ultimo_jpg
        if jpg is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.03)


@app.route("/video")
def video():
    return Response(generador_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    threading.Thread(target=bucle_camara, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
