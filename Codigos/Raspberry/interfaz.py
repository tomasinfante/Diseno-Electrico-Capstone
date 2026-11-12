#!/usr/bin/env python3
"""
Interfaz de seguimiento - Raspberry Pi 4 + Camera Module 2
Dashboard web con:
    1. Video en tiempo real con el objetivo detectado
    2. Error de posición del objetivo respecto al centro de la imagen (px y grados)
    3. Definición (resolución) de la imagen que se está recibiendo
    4. FPS del video

Instalación (Raspberry Pi OS Bookworm):
    sudo apt install -y python3-picamera2 python3-opencv python3-flask python3-serial

    5. Botones para cambiar la etapa del programa (Detenido, Calibrar, Búsqueda)

Etapas (la lógica de control la hace la ESP; aquí solo se le informa la etapa):
    1. Detenido     \
    2. Calibrar      > se eligen con los botones de la página
    3. Búsqueda     /
    4. Seguimiento  -> automática: desde Búsqueda, al detectar el objetivo por primera vez
El objetivo a seguir (la figura triangular o el círculo de un láser rojo) y los parámetros
de detección solo se pueden cambiar en la etapa Calibrar.

Instalación de la UART:
    sudo raspi-config -> Interface Options -> Serial Port
Paquete de error, 6 bytes: 0xAA | etapa (0x01..0x04) | ex (int16) | ey (int16), little-endian, en [px].
Paquete de parámetros del control, 13 bytes: 0xAA | 0x10 | 5 valores uint16 en milésimas | suma.
"""

import json
import struct
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, request

# ---------------- Parámetros ----------------
ANCHO, ALTO = 640, 480
FPS_CAMARA = 50
JPEG_CALIDAD = 40
# El video del dashboard se dibuja y codifica en un hilo aparte, a lo más a este ritmo, para
# que no le quite tiempo a la detección (que va al ritmo de la cámara).
VIDEO_FPS_MAX = 25
PUERTO = 5000


# (1640, 1232) = campo de visión completo con binning 2x2 (hasta 81 FPS).
MODO_SENSOR = (1640, 1232)

#FOV
FOV_H, FOV_V = 62.2, 48.8

# Color del objetivo. En OpenCV el tono (H) va de 0 a 179 y el rojo está en ambos extremos:
# se acepta desde TONO_ROJO_MIN hasta 179 y desde 0 hasta TONO_MAX.
# El tono es lo que separa al rojo de la piel: con esta cámara el rojo queda entre 175 y 2,
# y la cara y las manos entre 8 y 18 con una saturación parecida a la del rojo. Por eso
# TONO_MAX es bajo: si sube de ~7 empieza a entrar la piel.
# El brillo mínimo es bajo a propósito: a contraluz el objetivo se ve oscuro, pero conserva
# su tono y su saturación.
TONO_ROJO_MIN = 168
TONO_MAX = 5        # tono máximo que cuenta como rojo (0 a 15)
# La saturación separa al avión de los labios, que tienen su mismo tono: en la cámara el avión
# mide ~200 y los labios y la piel más rojiza no pasan de ~157.
SAT_MIN = 160       # saturación mínima (0 a 255): descarta grises, blancos, labios y piel
# En el borde del objetivo el color se mezcla con el fondo y la saturación baja. Para no
# recortarlo (importa cuando se ve chico), una mancha puede incluir píxeles hasta SAT_BORDE
# por debajo de SAT_MIN, siempre que al menos NUCLEO_FRAC de ella sí llegue a SAT_MIN.
SAT_BORDE = 50
NUCLEO_FRAC = 0.25
BRILLO_MIN = 40     # brillo mínimo (0 a 255): descarta lo casi negro, donde el tono es puro ruido

# Compensación de exposición de la cámara [EV]: positiva aclara la imagen. Sirve cuando el
# objetivo está a contraluz y queda tan oscuro que ya no se distingue su color.
EXPOSICION_EV = 0.0

VER_MASCARA = False  # pinta en el video los píxeles que cuentan como rojo, para calibrar

# Objetivo aproximado como triángulo isósceles: base 81.66 mm, altura 100 mm
TRI_BASE_MM = 81.66
TRI_ALTURA_MM = 100.0
PROPORCION = TRI_BASE_MM / TRI_ALTURA_MM  # base/altura esperada (≈ 0.82)
# ±25 %: con el avión real la proporción medida va de 0.77 a 1.0 según el tamaño y el giro
PROPORCION_TOL = 0.25   # desviación relativa aceptada en base/altura
ISOSCELES_TOL = 0.25    # diferencia relativa aceptada entre los dos lados largos
# El objetivo es la silueta de un avión: tiene entrantes entre las alas y el fuselaje, así que
# llena solo una parte del triángulo que la encierra. Por eso el relleno mínimo es bajo y no
# se exige que la forma sea convexa. Subirlo descarta más formas, pero también al avión.
RELLENO_MIN = 0.40      # área del contorno / área del triángulo que lo encierra
# El avión ocupa cerca de la mitad del rectángulo que lo encierra (0.51 a 0.54 medido); un
# óvalo (labios, una cara) ocupa 0.79 y un rectángulo 1.0
OCUPACION_MAX = 0.70    # área del contorno / área del rectángulo que lo encierra

AREA_MIN = 80         # px^2; con menos píxeles la forma no es confiable

# Objetivo alternativo: el círculo que produce un láser rojo.
# El centro del punto suele saturar el sensor y verse casi blanco
NUCLEO_BAJO = np.array([0, 0, 230])
NUCLEO_ALTO = np.array([179, 80, 255])
AREA_MIN_LASER = 1       # px^2, descarta ruido
CIRCULARIDAD_MIN = 0.6   # 1.0 = círculo perfecto

# Continuidad entre cuadros: evita saltar a otra cosa y las detecciones de un solo cuadro
FRAMES_CONFIRMAR = 3    # cuadros seguidos en que debe verse un objetivo nuevo antes de aceptarlo
MEMORIA_S = 0.3         # s; si el objetivo desaparece, durante este tiempo no se acepta otro lejos de él
PUERTA_BASE = 0.10      # distancia máxima entre cuadros, en anchos de imagen...
PUERTA_POR_S = 3.0      # ...más lo que puede moverse por segundo, también en anchos de imagen

# Qué se sigue: "triangulo" (la figura triangular) o "laser" (el círculo del láser)
OBJETIVOS = ("triangulo", "laser")
OBJETIVO = "triangulo"

# Parámetros del control que corre en la ESP: se le envían por la UART
OMEGA_MAX = 3.0            # velocidad máxima de los ejes [rad/s]
KP_PAN, KI_PAN = 2.0, 0.5  # ganancias del PI de cada eje: Kp [1/s], Ki [1/s²]
KP_TILT, KI_TILT = 2.0, 0.5

# Todo lo anterior (OBJETIVO, los parámetros de detección y los del control) se cambia desde
# la página web: OMEGA_MAX en cualquier etapa y el resto solo en la etapa Calibrar. Los valores
# guardados ahí quedan en este archivo y reemplazan a los de arriba al iniciar.
ARCHIVO_AJUSTES = Path(__file__).with_name("ajustes.json")

HISTORIAL_MAX = 600     # muestras guardadas para los gráficos (~20 s a 30 FPS)

# UART: GPIO14 (TX, pin 8), GPIO15 (RX, pin 10), GND (pin 6)
UART_PUERTO = "/dev/serial0"
UART_BAUDIOS = 115200
UART_CABECERA = 0xAA    # primer byte; el segundo es la etapa o UART_TIPO_PARAMETROS
UART_TIPO_PARAMETROS = 0x10
PERIODO_PARAMETROS = 1.0  # s; los parámetros del control se reenvían por si la ESP se reinicia

# Etapas del programa: su número es el segundo byte de la cabecera UART
DETENIDO, CALIBRAR, BUSQUEDA, SEGUIMIENTO = 1, 2, 3, 4
ETAPAS = {DETENIDO: "Detenido", CALIBRAR: "Calibrar", BUSQUEDA: "Búsqueda", SEGUIMIENTO: "Seguimiento"}
ETAPAS_MANUALES = (DETENIDO, CALIBRAR, BUSQUEDA)  # las que se eligen con los botones

# ---------------- Estado compartido ----------------
etapa = DETENIDO  # se lee y se cambia siempre con `lock` tomado
parametros_cambiados = threading.Event()  # hay que enviar ya los parámetros del control a la ESP
lock = threading.Lock()
nuevo_frame = threading.Condition(lock)   # hay un JPEG nuevo para el video
ultimo_jpg = None
id_video = 0                               # cuenta los JPEG publicados
id_frame = 0                               # cuenta los cuadros procesados (detección)
info_video = {"jpg_kb": 0.0, "fps_video": 0.0, "video_ms": 0.0}
hay_cuadro = threading.Condition()         # la detección dejó un cuadro para el hilo del video
cuadro_video = None                        # (frame, objetivo, error, fps, mascara, descartes)
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

    def fijar_ev(self, ev):
        """Compensación de exposición [EV]: positiva aclara la imagen, negativa la oscurece."""
        try:
            self.picam2.set_controls({"ExposureValue": float(ev)})
        except Exception as e:  # la cámara sigue funcionando con la exposición que tenía
            print(f"No se pudo fijar la compensación de exposición ({e})")

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




# ---------------- Visión ----------------
def tono_rojo(hsv, sat_min):
    return (cv2.inRange(hsv, (0, sat_min, BRILLO_MIN), (TONO_MAX, 255, 255))
            | cv2.inRange(hsv, (TONO_ROJO_MIN, sat_min, BRILLO_MIN), (179, 255, 255)))


def mascara_roja(hsv):
    """
    Píxeles que cuentan como rojo: tono cerca de 0, con saturación y brillo suficientes.
    Se arma en dos niveles: el rojo "fuerte" (saturación >= SAT_MIN) decide qué manchas
    valen, y cada una se completa con el rojo "débil" que la rodea (hasta SAT_BORDE menos).
    Una mancha solo de rojo débil, como los labios, queda fuera.
    """
    debil = tono_rojo(hsv, max(SAT_MIN - SAT_BORDE, 0))

    # El resto se hace solo en el rectángulo que contiene al rojo (suele ser una zona chica)
    x, y, w, h = cv2.boundingRect(debil)
    if w == 0:
        return debil
    zona = debil[y:y + h, x:x + w]
    fuerte = (hsv[y:y + h, x:x + w, 1] >= SAT_MIN) & (zona > 0)

    n, etiquetas = cv2.connectedComponents(zona)
    total = np.bincount(etiquetas.ravel(), minlength=n)
    nucleo = np.bincount(etiquetas[fuerte], minlength=n)
    vale = nucleo >= NUCLEO_FRAC * total
    vale[0] = False  # la etiqueta 0 es el fondo
    if not vale.all():
        zona[~vale[etiquetas]] = 0
    return debil


def detectar(frame_bgr):
    """
    Busca en el cuadro los candidatos a ser el objetivo elegido en la etapa Calibrar (OBJETIVO):
    el triángulo rojo o el círculo del láser rojo. Devuelve (candidatos, mascara, descartes):
      - candidatos: lista de dicts con al menos "tipo", "x", "y" [px] y "area" [px²]. Cuál de
        ellos es el objetivo lo decide SeguidorObjetivo.
      - mascara: los píxeles que cuentan como rojo.
      - descartes: manchas rojas rechazadas por su forma, como (x, y, motivo), para mostrarlas
        en el video junto con la máscara y saber qué parámetro ajustar.
    """
    blur = cv2.GaussianBlur(frame_bgr, (7, 7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    rojo = mascara_roja(hsv)

    descartes = []
    if OBJETIVO == "laser":
        return detectar_laser(hsv, rojo, descartes), rojo, descartes
    return detectar_triangulo(rojo, descartes), rojo, descartes


def contornos_de(mascara):
    """Limpia la máscara (quita motas y cierra huecos) y devuelve sus contornos externos."""
    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel, iterations=1)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel, iterations=2)
    return cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]


def centro_de(contorno):
    x, y, w, h = cv2.boundingRect(contorno)
    return x + w // 2, y + h // 2


def detectar_laser(hsv, rojo, descartes):
    """
    Candidatos a punto del láser rojo: manchas rojas aproximadamente circulares.
    Cada uno es un dict con el centro (x, y), el radio [px], el área y la circularidad medida.
    """
    # Núcleo blanco saturado: solo se acepta si está rodeado de rojo
    kernel = np.ones((5, 5), np.uint8)
    nucleo = cv2.inRange(hsv, NUCLEO_BAJO, NUCLEO_ALTO) & cv2.dilate(rojo, kernel, iterations=2)

    candidatos = []
    for c in contornos_de(rojo | nucleo):
        area = cv2.contourArea(c)
        if area < AREA_MIN_LASER:
            continue
        perimetro = cv2.arcLength(c, True)
        if perimetro == 0:
            continue
        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad < CIRCULARIDAD_MIN:
            descartes.append((*centro_de(c), f"circularidad {circularidad:.2f}"))
            continue

        (x, y), radio = cv2.minEnclosingCircle(c)
        candidatos.append({
            "tipo": "laser",
            "x": float(x),
            "y": float(y),
            "area": float(area),
            "radio": float(radio),
            "circularidad": float(circularidad),
        })

    return candidatos


def detectar_triangulo(rojo, descartes):
    """
    Candidatos a objetivo rojo con forma de triángulo (base/altura ≈ PROPORCION).
    Cada uno es un dict con el centroide (x, y), el área, el triángulo [3 vértices], el
    vértice de la punta, base y altura [px] y la proporción medida.
    """
    candidatos = []
    for c in contornos_de(rojo):
        area = cv2.contourArea(c)
        if area < AREA_MIN:
            continue

        area_tri, tri = cv2.minEnclosingTriangle(cv2.convexHull(c))
        if area_tri <= 0 or area / area_tri < RELLENO_MIN:
            descartes.append((*centro_de(c), f"relleno {area / area_tri:.2f}" if area_tri > 0 else "sin forma"))
            continue
        tri = tri.reshape(3, 2)

        # Proporción base/altura con el rectángulo más chico que encierra al contorno (en
        # cualquier giro): lado corto = base (envergadura), lado largo = altura (largo).
        # El triángulo envolvente no sirve para esto: con la silueta real del avión no queda
        # ajustado a la envergadura y al largo, y da proporciones cercanas a 1.
        ancho_rect, alto_rect = cv2.minAreaRect(c)[1]
        base, altura = min(ancho_rect, alto_rect), max(ancho_rect, alto_rect)
        proporcion = base / altura if altura > 0 else 0.0
        ocupacion = area / (base * altura) if base > 0 else 1.0
        if ocupacion > OCUPACION_MAX:
            descartes.append((*centro_de(c), f"rect {ocupacion:.2f}"))
            continue
        if abs(proporcion / PROPORCION - 1) > PROPORCION_TOL:
            descartes.append((*centro_de(c), f"b/h {proporcion:.2f}"))
            continue

        # lados[i] = lado opuesto al vértice i del triángulo envolvente. Su lado más corto
        # hace de base y el vértice opuesto es la punta.
        lados = [float(np.linalg.norm(tri[(i + 1) % 3] - tri[(i + 2) % 3])) for i in range(3)]
        i_punta = int(np.argmin(lados))
        largo1, largo2 = (lados[j] for j in range(3) if j != i_punta)
        if abs(largo1 - largo2) / max(largo1, largo2) > ISOSCELES_TOL:
            descartes.append((*centro_de(c), f"lados {min(largo1, largo2) / max(largo1, largo2):.2f}"))
            continue

        mom = cv2.moments(c)
        candidatos.append({
            "tipo": "triangulo",
            "x": mom["m10"] / mom["m00"],
            "y": mom["m01"] / mom["m00"],
            "area": float(area),
            "tri": tri,
            "punta": tri[i_punta],
            "base": base,
            "altura": altura,
            "proporcion": proporcion,
        })

    return candidatos


class SeguidorObjetivo:
    """
    Decide cuál de los candidatos de cada cuadro es el objetivo, usando lo visto antes:
      - Un objetivo nuevo solo se acepta tras verlo FRAMES_CONFIRMAR cuadros seguidos en
        (casi) el mismo lugar: una detección falsa de un solo cuadro no cuenta.
      - Mientras se sigue un objetivo solo se acepta el candidato más cercano a su última
        posición, dentro de una distancia máxima ("puerta"): no se salta a otra cosa aunque
        sea más grande. Si desaparece, se lo espera MEMORIA_S antes de buscar uno nuevo.
    """

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.pos = None        # última posición aceptada (x, y)
        self.t = 0.0           # instante de esa posición
        self.pendiente = None  # posición del candidato que se está confirmando
        self.cuenta = 0        # cuadros seguidos en que se vio

    @staticmethod
    def _distancia(candidato, pos):
        return float(np.hypot(candidato["x"] - pos[0], candidato["y"] - pos[1]))

    def elegir(self, candidatos, t, ancho):
        """Devuelve el candidato que es el objetivo en el instante t [s], o None."""
        if self.pos is not None and t - self.t <= MEMORIA_S:
            # La puerta crece con el tiempo sin verlo: pudo moverse más
            puerta = ancho * (PUERTA_BASE + PUERTA_POR_S * (t - self.t))
            cerca = [c for c in candidatos if self._distancia(c, self.pos) <= puerta]
            if not cerca:
                return None
            elegido = min(cerca, key=lambda c: self._distancia(c, self.pos))
            self.pos, self.t = (elegido["x"], elegido["y"]), t
            return elegido

        # No se sigue a nadie: confirmar al candidato más grande
        self.pos = None
        if not candidatos:
            self.pendiente, self.cuenta = None, 0
            return None
        mayor = max(candidatos, key=lambda c: c["area"])
        cerca = self.pendiente is not None and self._distancia(mayor, self.pendiente) <= ancho * 2 * PUERTA_BASE
        self.cuenta = self.cuenta + 1 if cerca else 1
        self.pendiente = (mayor["x"], mayor["y"])
        if self.cuenta < FRAMES_CONFIRMAR:
            return None
        self.pos, self.t = self.pendiente, t
        self.pendiente, self.cuenta = None, 0
        return mayor


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


def dibujar(frame, objetivo, error, fps, mascara=None, descartes=()):
    alto, ancho = frame.shape[:2]
    cx, cy = ancho // 2, alto // 2
    g = max(1, round(ancho / 640))  # grosor proporcional a la resolución

    if mascara is not None:
        # Píxeles que cuentan como rojo, en celeste: para ajustar tono, saturación y brillo
        rojos = mascara > 0
        frame[rojos] = frame[rojos] // 3 + np.array([170, 170, 0], np.uint8)
        # Manchas rojas rechazadas por su forma, con el valor que no cumplió
        for x, y, motivo in descartes:
            etiqueta(frame, motivo, x + 8 * g, y - 8 * g, 0.4 * g, g)

    cv2.drawMarker(frame, (cx, cy), (220, 220, 220), cv2.MARKER_CROSS, 24 * g, g)

    if objetivo:
        x, y = int(round(objetivo["x"])), int(round(objetivo["y"]))
        largo = max(np.hypot(x - cx, y - cy), 1)
        cv2.arrowedLine(frame, (cx, cy), (x, y), (0, 220, 255), g, cv2.LINE_AA,
                        tipLength=min(0.3, 12 * g / largo))
        if objetivo["tipo"] == "laser":
            r = int(round(objetivo["radio"]))
            cv2.circle(frame, (x, y), r, (255, 0, 255), 2 * g, cv2.LINE_AA)
            x_min, x_max = x - r, x + r
        else:
            tri = np.round(objetivo["tri"]).astype(np.int32)
            cv2.polylines(frame, [tri], True, (255, 0, 255), 2 * g, cv2.LINE_AA)
            px, py = (int(round(v)) for v in objetivo["punta"])
            cv2.circle(frame, (px, py), 4 * g, (255, 255, 0), -1)  # punta del triángulo
            x_min, x_max = int(tri[:, 0].min()), int(tri[:, 0].max())
        cv2.circle(frame, (x, y), 3 * g, (255, 0, 255), -1)

        # El texto va a la derecha del objetivo, o a la izquierda si no cabe
        texto = f"({error[0]:+.0f}, {error[1]:+.0f}) px"
        tw = cv2.getTextSize(texto, FUENTE, 0.5 * g, g)[0][0] + 6 * g
        tx = x_max + 6 if x_max + 6 + tw < ancho else x_min - 6 - tw
        etiqueta(frame, texto, tx, y - 10 * g, 0.5 * g, g)

    etiqueta(frame, f"{ancho}x{alto}  {fps:.1f} FPS", 6, 6, 0.55 * g, g)


# ---------------- UART ----------------
def abrir_uart():
    """Abre el puerto serie. Si falla, la interfaz sigue funcionando sin enviar."""
    try:
        return serial.Serial(UART_PUERTO, UART_BAUDIOS, timeout=0, write_timeout=0.01)
    except serial.SerialException as e:
        print(f"UART no disponible ({e}): no se enviarán los errores")
        return None


def enviar_uart(uart, etapa, ex, ey):
    """
    Envía la etapa y la parte entera de ex y ey [px] en binario, en un paquete de 6 bytes:
        0xAA | etapa (0x01..0x04) | ex (int16) | ey (int16)   — little-endian, con signo
    El 0xAA permite al receptor sincronizarse con el inicio de cada paquete y el
    segundo byte de la cabecera le indica la etapa del programa.
    """
    escribir_uart(uart, struct.pack("<BBhh", UART_CABECERA, etapa, int(ex), int(ey)))


def paquete_parametros():
    """
    Paquete de 13 bytes con los parámetros del control:
        0xAA | 0x10 | omegaMax | KpPan | KiPan | KpTilt | KiTilt | suma
    Cada valor es un uint16 little-endian en milésimas (2500 = 2.5). `suma` es la suma, módulo
    256, de los 11 bytes anteriores (desde el 0x10): la ESP descarta el paquete si no coincide.
    """
    valores = [round(v * 1000) for v in (OMEGA_MAX, KP_PAN, KI_PAN, KP_TILT, KI_TILT)]
    cuerpo = struct.pack("<B5H", UART_TIPO_PARAMETROS, *valores)
    return bytes([UART_CABECERA]) + cuerpo + bytes([sum(cuerpo) & 0xFF])


def escribir_uart(uart, datos):
    if uart is None:
        return
    try:
        uart.write(datos)
    except serial.SerialTimeoutException:
        pass  # buffer lleno: se descarta este paquete para no frenar el video


def bucle_video():
    """
    Dibuja y codifica en JPEG el cuadro más reciente que dejó la detección. Va en su propio
    hilo: si no alcanza a procesar todos los cuadros se salta los intermedios, sin atrasar
    a la detección ni a la UART.
    """
    global ultimo_jpg, id_video, info_video, cuadro_video

    publicados = deque()  # instantes de publicación del último segundo
    while True:
        with hay_cuadro:
            hay_cuadro.wait_for(lambda: cuadro_video is not None)
            frame, objetivo, error, fps, mascara, descartes = cuadro_video
            cuadro_video = None

        t_ini = time.perf_counter()
        dibujar(frame, objetivo, error, fps, mascara, descartes)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_CALIDAD])
        t_fin = time.perf_counter()
        if ok:
            publicados.append(t_fin)
            while t_fin - publicados[0] > 1.0:
                publicados.popleft()
            fps_video = (len(publicados) - 1) / (t_fin - publicados[0]) if len(publicados) > 1 else 0.0
            with lock:
                ultimo_jpg = jpg.tobytes()
                id_video += 1
                info_video = {"jpg_kb": round(len(jpg) / 1024, 1), "fps_video": round(fps_video, 1),
                              "video_ms": round((t_fin - t_ini) * 1000, 1)}
                nuevo_frame.notify_all()

        # No más de VIDEO_FPS_MAX: el resto del tiempo queda libre para la detección
        espera = 1.0 / VIDEO_FPS_MAX - (time.perf_counter() - t_ini)
        if espera > 0:
            time.sleep(espera)


def bucle_camara():
    global id_frame, estado, etapa, cuadro_video

    camara = CamaraPi()
    uart = abrir_uart()
    t0 = time.time()
    t_parametros = 0.0  # último envío de los parámetros del control
    seguidor = SeguidorObjetivo()
    tipo_seguido = OBJETIVO
    ev_aplicado = 0.0
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

        if EXPOSICION_EV != ev_aplicado:
            ev_aplicado = EXPOSICION_EV
            camara.fijar_ev(ev_aplicado)
        if OBJETIVO != tipo_seguido:  # se eligió otro objetivo: se olvida al anterior
            tipo_seguido = OBJETIVO
            seguidor.reiniciar()

        candidatos, mascara, descartes = detectar(frame)
        objetivo = seguidor.elegir(candidatos, t, ancho)
        error = calcular_error(objetivo["x"], objetivo["y"], ancho, alto, meta["fov"]) if objetivo else None

        # Búsqueda -> Seguimiento apenas aparece el objetivo (ya confirmado por el seguidor)
        with lock:
            if etapa == BUSQUEDA and objetivo:
                etapa = SEGUIMIENTO
                print("Objetivo detectado: Búsqueda -> Seguimiento")
            etapa_frame = etapa

        # La UART va antes de dibujar y codificar el JPEG, para no atrasar el control de la ESP
        ex, ey = (round(error[0], 1), round(error[1], 1)) if error else (None, None)
        # Parámetros del control: apenas cambian en la página y además cada PERIODO_PARAMETROS
        if parametros_cambiados.is_set() or t - t_parametros >= PERIODO_PARAMETROS:
            parametros_cambiados.clear()
            t_parametros = t
            escribir_uart(uart, paquete_parametros())

        if etapa_frame != SEGUIMIENTO:
            # Detenido, Calibrar y Búsqueda: se informa la etapa en cada frame, con error 0
            enviar_uart(uart, etapa_frame, 0, 0)
        elif error:
            # Seguimiento: solo con objetivo a la vista (la ESP detecta su pérdida por timeout)
            enviar_uart(uart, etapa_frame, ex, ey)

        # El dibujo y el JPEG los hace bucle_video con el cuadro más reciente
        with hay_cuadro:
            cuadro_video = (frame, objetivo, error, fps, mascara if VER_MASCARA else None,
                            descartes if VER_MASCARA else ())
            hay_cuadro.notify()
        proc_ms = (time.perf_counter() - t_ini) * 1000

        nuevo_estado = {
            **camara.info,
            "etapa": etapa_frame,
            "detectado": objetivo is not None,
            "x": round(objetivo["x"], 1) if objetivo else None,
            "y": round(objetivo["y"], 1) if objetivo else None,
            "objetivo": OBJETIVO,
            # Según el tipo de objetivo: base, altura y proporción (triángulo) o radio y circularidad (láser)
            **{k: round(objetivo[k], n) if objetivo and k in objetivo else None
               for k, n in (("base", 1), ("altura", 1), ("proporcion", 2), ("radio", 1), ("circularidad", 2))},
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
            "jpeg_calidad": JPEG_CALIDAD,
        }

        with lock:
            id_frame += 1
            estado = {**nuevo_estado, **info_video}
            historial.append([id_frame, round(t - t0, 3), objetivo is not None, ex, ey, round(fps, 1)])


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
#ajustes{margin-top:16px}
.campos{display:flex;flex-wrap:wrap;gap:16px;align-items:flex-end}
.campo{display:flex;flex-direction:column;gap:4px;min-width:180px}
.campo span{font-size:12px;color:var(--muted)}
.campo input{font:inherit;font-variant-numeric:tabular-nums;color:var(--text);background:var(--bg);
             border:1px solid var(--border);border-radius:6px;padding:6px 8px;width:100%}
.campo small{font-size:12px;color:var(--text-2);font-variant-numeric:tabular-nums}
.botones{display:flex;gap:8px}
button{font:inherit;padding:6px 14px;border-radius:6px;border:1px solid var(--border);
       background:var(--surface);color:var(--text);cursor:pointer}
button.principal{background:var(--s1);border-color:var(--s1);color:#fff}
fieldset{border:0;padding:0;margin:0;min-width:0;display:flex;flex-direction:column;gap:16px}
fieldset:disabled{opacity:.55}
fieldset:disabled button,fieldset:disabled input{cursor:not-allowed}
.opciones{display:flex;flex-wrap:wrap;gap:8px}
.opciones button.activa{background:var(--s1);border-color:var(--s1);color:#fff;font-weight:600}
#etapas{margin-bottom:16px}
#control{margin-top:16px}
.casilla{display:flex;align-items:center;gap:8px;margin-bottom:12px;font-size:13px;color:var(--text-2)}
#control fieldset{margin-top:16px}
.etapas{display:flex;flex-wrap:wrap;gap:8px}
.etapas button{padding:8px 18px}
.etapas button:disabled{cursor:default;opacity:.55}
.etapas button.activa{background:var(--s1);border-color:var(--s1);color:#fff;font-weight:600;opacity:1}
</style></head><body>
<header>
  <h1>Interfaz de seguimiento</h1>
  <span class="pill" id="estado"><span class="ico">…</span> Esperando datos</span>
  <span class="pill"><span class="dot" id="conn-dot"></span><span id="conn">Conectando</span></span>
</header>

<section class="card" id="etapas">
  <h2>Etapa del programa <span>Seguimiento se activa sola al detectar el objetivo durante la búsqueda</span></h2>
  <div class="etapas">
    <button type="button" data-etapa="1">1 · Detenido</button>
    <button type="button" data-etapa="2">2 · Calibrar</button>
    <button type="button" data-etapa="3">3 · Búsqueda</button>
    <button type="button" data-etapa="4" disabled title="Automática: no se elige a mano">4 · Seguimiento</button>
  </div>
  <p class="nota" id="etapa-msg"></p>
</section>

<div class="grid">
  <div class="col">
    <section class="card">
      <h2>Video en tiempo real <span class="num" id="tag-res">—</span></h2>
      <img id="video" src="/video" alt="Video de la cámara con el objetivo detectado">
      <p class="nota">Cruz = centro de la imagen · contorno magenta = objetivo · flecha = error de posición · celeste = píxeles que cuentan como rojo (con la máscara activada)</p>
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
      <h2>FPS de la detección</h2>
      <div class="hero"><b id="fps">—</b><span>FPS medidos</span></div>
      <div class="sub">Nominal cámara <b id="fps-nom">—</b> · Detección <b id="proc">—</b> ms/frame</div>
      <div class="sub">Video del dashboard <b id="fps-video">—</b> FPS · <b id="video-ms">—</b> ms/frame</div>
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

<section class="card" id="control">
  <h2>Control de los motores <span>se envían a la ESP al guardarlos</span></h2>
  <form id="form-omega" class="campos">
    <label class="campo"><span>Velocidad máxima (OMEGA_MAX) [rad/s]</span>
      <input type="number" id="omega-max" min="0.1" max="10" step="0.05" required>
      <small id="omega-grados">—</small></label>
    <div class="botones"><button type="submit" class="principal">Aplicar</button></div>
  </form>
  <p class="nota">La velocidad máxima se puede cambiar en cualquier etapa; las ganancias, solo en la etapa Calibrar.</p>
  <fieldset class="solo-calibrar" disabled>
    <form id="form-ganancias" class="campos">
      <label class="campo"><span>Pan · Kp [1/s]</span>
        <input type="number" id="kp-pan" min="0" max="50" step="0.01" required></label>
      <label class="campo"><span>Pan · Ki [1/s²]</span>
        <input type="number" id="ki-pan" min="0" max="50" step="0.01" required></label>
      <label class="campo"><span>Tilt · Kp [1/s]</span>
        <input type="number" id="kp-tilt" min="0" max="50" step="0.01" required></label>
      <label class="campo"><span>Tilt · Ki [1/s²]</span>
        <input type="number" id="ki-tilt" min="0" max="50" step="0.01" required></label>
      <div class="botones"><button type="submit" class="principal">Guardar ganancias</button></div>
    </form>
  </fieldset>
  <p class="nota" id="control-msg"></p>
</section>

<section class="card" id="ajustes">
  <h2>Calibración <span id="ajustes-estado">solo se puede modificar en la etapa Calibrar</span></h2>
  <label class="casilla"><input type="checkbox" id="ver-mascara">
    Ver en el video los píxeles que cuentan como rojo y por qué se descarta cada mancha
    (se puede activar en cualquier etapa)</label>
  <p class="nota" style="margin:0 0 12px">Etiquetas sobre una mancha roja descartada: «relleno» está bajo el relleno mínimo ·
    «rect» ocupa demasiado del rectángulo que la encierra (óvalos y rectángulos, sobre 0,70) ·
    «b/h» es su proporción base/altura, fuera de la tolerancia · «lados» son sus dos lados largos, muy distintos ·
    «circularidad» está bajo la mínima del láser. Las manchas más chicas que el área mínima no llevan etiqueta.</p>
  <fieldset class="solo-calibrar" disabled>
    <div class="campo"><span>Objetivo a seguir</span>
      <div class="opciones" id="objetivos">
        <button type="button" data-objetivo="triangulo">Figura triangular</button>
        <button type="button" data-objetivo="laser">Círculo del láser</button>
      </div></div>
    <form id="form-ajustes" class="campos">
      <label class="campo"><span>Triángulo · área mínima (AREA_MIN) [px²]</span>
        <input type="number" id="area-min" min="1" step="1" required>
        <small>Manchas rojas más pequeñas se descartan</small></label>
      <label class="campo"><span>Triángulo · tolerancia de proporción (PROPORCION_TOL)</span>
        <input type="number" id="prop-tol" min="0.01" max="1" step="0.01" required>
        <small id="rango-prop">—</small></label>
      <label class="campo"><span>Láser · área mínima (AREA_MIN_LASER) [px²]</span>
        <input type="number" id="area-min-laser" min="1" step="1" required>
        <small>Manchas rojas más pequeñas se descartan</small></label>
      <label class="campo"><span>Láser · circularidad mínima (CIRCULARIDAD_MIN)</span>
        <input type="number" id="circ-min" min="0.01" max="1" step="0.01" required>
        <small>1 = círculo perfecto</small></label>
      <label class="campo"><span>Rojo · tono máximo (TONO_MAX)</span>
        <input type="number" id="tono-max" min="0" max="15" step="1" required>
        <small>0 a 15 · sobre ~7 empieza a entrar la piel</small></label>
      <label class="campo"><span>Rojo · saturación mínima (SAT_MIN)</span>
        <input type="number" id="sat-min" min="0" max="255" step="1" required>
        <small>0 a 255 · avión ≈ 200, labios y piel hasta ≈ 157 · los bordes aceptan hasta 50 menos</small></label>
      <label class="campo"><span>Rojo · brillo mínimo (BRILLO_MIN)</span>
        <input type="number" id="brillo-min" min="0" max="255" step="1" required>
        <small>0 a 255 · bajarlo si el objetivo está a contraluz</small></label>
      <label class="campo"><span>Triángulo · relleno mínimo (RELLENO_MIN)</span>
        <input type="number" id="relleno-min" min="0.3" max="1" step="0.01" required>
        <small>parte del triángulo envolvente que llena el objetivo · subirlo puede descartar al avión</small></label>
      <label class="campo"><span>Cámara · compensación de exposición [EV]</span>
        <input type="number" id="exposicion-ev" min="-4" max="4" step="0.25" required>
        <small>positiva aclara: para objetivos a contraluz</small></label>
      <div class="botones">
        <button type="submit" class="principal">Guardar</button>
        <button type="button" id="restablecer">Valores por defecto</button>
      </div>
    </form>
  </fieldset>
  <p class="nota" id="ajustes-msg"></p>
</section>

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
function pintarEtapa(n) {
  for (const b of document.querySelectorAll('#etapas button'))
    b.classList.toggle('activa', Number(b.dataset.etapa) === n);
  const calibrando = n === 2;
  for (const f of document.querySelectorAll('fieldset.solo-calibrar')) f.disabled = !calibrando;
  $('ajustes-estado').textContent = calibrando
    ? 'los cambios se aplican al instante y quedan guardados'
    : 'solo se puede modificar en la etapa Calibrar';
}

async function cambiarEtapa(n) {
  const msg = $('etapa-msg');
  try {
    const r = await fetch('/etapa', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                      body: JSON.stringify({etapa: n})});
    const d = await r.json();
    if (!r.ok) { msg.textContent = 'Error: ' + d.error; return; }
    pintarEtapa(d.etapa);
    msg.textContent = '';
  } catch (e) {
    msg.textContent = 'No se pudo cambiar la etapa: sin conexión con la Raspberry';
  }
}

for (const b of document.querySelectorAll('#etapas button:not(:disabled)'))
  b.addEventListener('click', () => cambiarEtapa(Number(b.dataset.etapa)));
fetch('/etapa').then(r => r.json()).then(d => pintarEtapa(d.etapa)).catch(() => {});

function pintarEstado(d) {
  pintarEtapa(d.etapa);
  const det = d.detectado;
  $('estado').innerHTML = det
    ? '<span class="ico ok">✓</span> Objetivo detectado'
    : '<span class="ico no">✕</span> Sin objetivo';

  $('ex').textContent = det ? firmado(d.ex) + ' px' : '—';
  $('ey').textContent = det ? firmado(d.ey) + ' px' : '—';
  $('em').textContent = det ? d.e_mag.toFixed(1) + ' px' : '—';
  $('ex-deg').textContent = det ? firmado(d.ex_deg, 2) + '°' : '—';
  $('ey-deg').textContent = det ? firmado(d.ey_deg, 2) + '°' : '—';
  const forma = d.proporcion != null ? ` b/h=${d.proporcion.toFixed(2)}`
              : d.radio != null ? ` r=${d.radio.toFixed(0)} px` : '';
  $('pos').textContent = det ? `(${d.x.toFixed(0)}, ${d.y.toFixed(0)})${forma}` : '—';

  $('fps').textContent = d.fps.toFixed(1);
  $('fps-nom').textContent = d.fps_nominal ? d.fps_nominal.toFixed(1) : '—';
  $('proc').textContent = d.proc_ms.toFixed(1);
  $('fps-video').textContent = d.fps_video.toFixed(1);
  $('video-ms').textContent = d.video_ms.toFixed(1);

  $('tag-res').textContent = `${d.ancho}×${d.alto}`;
  $('res').textContent = `${d.ancho} × ${d.alto} px`;
  $('mp').textContent = `${(d.ancho * d.alto / 1e6).toFixed(2)} MP · ${aspecto(d.ancho, d.alto)}`;
  $('sensor').textContent = par(d.sensor);
  $('modo').textContent = par(d.modo_sensor);
  $('recorte').textContent = d.recorte && d.sensor
    ? `${par(d.recorte)} (${(100 * d.recorte[0] / d.sensor[0]).toFixed(0)} % del ancho)` : '—';
  $('fov').textContent = `${d.fov_supuesto ? '≈ ' : ''}${d.fov[0].toFixed(1)}° × ${d.fov[1].toFixed(1)}°`
    + (d.fov_supuesto ? ' (supuesto)' : '');
  $('jpg').textContent = `${d.jpg_kb.toFixed(1)} kB/frame · calidad ${d.jpeg_calidad} · ≈ ${(d.jpg_kb * d.fps_video / 1024).toFixed(2)} MB/s`;
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

// ---------- Parámetros de detección ----------
let ajustes = null;

function mostrarRango() {
  const tol = parseFloat($('prop-tol').value);
  if (!ajustes || !(tol > 0)) { $('rango-prop').textContent = '—'; return; }
  const p = ajustes.proporcion;
  $('rango-prop').textContent =
    `±${(tol * 100).toFixed(0)} % · acepta b/h entre ${(p * (1 - tol)).toFixed(2)} y ${(p * (1 + tol)).toFixed(2)}`;
}

function mostrarOmegaGrados() {
  const w = parseFloat($('omega-max').value);
  $('omega-grados').textContent = w > 0 ? `= ${(w * 180 / Math.PI).toFixed(0)} °/s` : '—';
}

function mostrarAjustes(a) {
  ajustes = a;
  $('area-min').value = a.area_min;
  $('prop-tol').value = a.proporcion_tol;
  $('area-min-laser').value = a.area_min_laser;
  $('circ-min').value = a.circularidad_min;
  $('tono-max').value = a.tono_max;
  $('sat-min').value = a.sat_min;
  $('brillo-min').value = a.brillo_min;
  $('relleno-min').value = a.relleno_min;
  $('exposicion-ev').value = a.exposicion_ev;
  $('ver-mascara').checked = a.ver_mascara;
  $('omega-max').value = a.omega_max;
  $('kp-pan').value = a.kp_pan;
  $('ki-pan').value = a.ki_pan;
  $('kp-tilt').value = a.kp_tilt;
  $('ki-tilt').value = a.ki_tilt;
  mostrarOmegaGrados();
  for (const b of document.querySelectorAll('#objetivos button'))
    b.classList.toggle('activa', b.dataset.objetivo === a.objetivo);
  mostrarRango();
}

async function enviarAjustes(valores, idMsg = 'ajustes-msg') {
  const msg = $(idMsg);
  try {
    const r = await fetch('/ajustes', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                        body: JSON.stringify(valores)});
    const d = await r.json();
    if (!r.ok) { msg.textContent = 'Error: ' + d.error; return; }
    mostrarAjustes(d);
    msg.textContent = `Guardado a las ${new Date().toLocaleTimeString()}`;
  } catch (e) {
    msg.textContent = 'No se pudo guardar: sin conexión con la Raspberry';
  }
}

$('prop-tol').addEventListener('input', mostrarRango);
$('form-ajustes').addEventListener('submit', e => {
  e.preventDefault();
  enviarAjustes({area_min: Number($('area-min').value), proporcion_tol: Number($('prop-tol').value),
                 area_min_laser: Number($('area-min-laser').value), circularidad_min: Number($('circ-min').value),
                 tono_max: Number($('tono-max').value), sat_min: Number($('sat-min').value),
                 brillo_min: Number($('brillo-min').value), relleno_min: Number($('relleno-min').value),
                 exposicion_ev: Number($('exposicion-ev').value)});
});
$('ver-mascara').addEventListener('change', e => enviarAjustes({ver_mascara: e.target.checked}));
for (const b of document.querySelectorAll('#objetivos button'))
  b.addEventListener('click', () => enviarAjustes({objetivo: b.dataset.objetivo}));
$('omega-max').addEventListener('input', mostrarOmegaGrados);
$('form-omega').addEventListener('submit', e => {
  e.preventDefault();
  enviarAjustes({omega_max: Number($('omega-max').value)}, 'control-msg');
});
$('form-ganancias').addEventListener('submit', e => {
  e.preventDefault();
  enviarAjustes({kp_pan: Number($('kp-pan').value), ki_pan: Number($('ki-pan').value),
                 kp_tilt: Number($('kp-tilt').value), ki_tilt: Number($('ki-tilt').value)}, 'control-msg');
});
$('restablecer').addEventListener('click', () => { if (ajustes) enviarAjustes(ajustes.defecto); });
fetch('/ajustes').then(r => r.json()).then(mostrarAjustes).catch(() => {});

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
            nuevo_frame.wait_for(lambda: id_video != visto, timeout=1.0)
            jpg, visto = ultimo_jpg, id_video
        if jpg is not None:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"


@app.route("/video")
def video():
    return Response(generador_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# ---------------- Etapa del programa ----------------
@app.route("/etapa", methods=["GET", "POST"])
def ruta_etapa():
    """GET: etapa actual. POST {"etapa": 1|2|3}: cambia a Detenido, Calibrar o Búsqueda."""
    global etapa
    if request.method == "POST":
        nueva = (request.get_json(silent=True) or {}).get("etapa")
        if type(nueva) is not int or nueva not in ETAPAS_MANUALES:
            return jsonify(error="la etapa debe ser 1 (Detenido), 2 (Calibrar) o 3 (Búsqueda)"), 400
        with lock:
            anterior, etapa = etapa, nueva
        if anterior != nueva:
            print(f"Etapa: {ETAPAS[anterior]} -> {ETAPAS[nueva]}")
    with lock:
        return jsonify(etapa=etapa, nombre=ETAPAS[etapa])


# ---------------- Ajustes editables desde la página ----------------
# Ajustes numéricos: clave -> (mínimo, máximo, decimales)
AJUSTES_NUMERICOS = {
    "area_min": (1, ANCHO * ALTO, 0),
    "proporcion_tol": (0.01, 1, 3),
    "area_min_laser": (1, ANCHO * ALTO, 0),
    "circularidad_min": (0.01, 1, 3),
    "tono_max": (0, 15, 0),
    "sat_min": (0, 255, 0),
    "brillo_min": (0, 255, 0),
    "relleno_min": (0.3, 1, 3),
    "exposicion_ev": (-4, 4, 2),
    "omega_max": (0.1, 10, 3),
    "kp_pan": (0, 50, 3),
    "ki_pan": (0, 50, 3),
    "kp_tilt": (0, 50, 3),
    "ki_tilt": (0, 50, 3),
}
AJUSTES_CONTROL = ("omega_max", "kp_pan", "ki_pan", "kp_tilt", "ki_tilt")  # los que se envían a la ESP
AJUSTES_LIBRES = ("omega_max", "ver_mascara")  # se pueden cambiar en cualquier etapa; el resto solo en Calibrar


def ajustes_actuales():
    return {"objetivo": OBJETIVO, "area_min": AREA_MIN, "proporcion_tol": PROPORCION_TOL,
            "area_min_laser": AREA_MIN_LASER, "circularidad_min": CIRCULARIDAD_MIN,
            "tono_max": TONO_MAX, "sat_min": SAT_MIN, "brillo_min": BRILLO_MIN,
            "relleno_min": RELLENO_MIN, "exposicion_ev": EXPOSICION_EV, "ver_mascara": VER_MASCARA,
            "omega_max": OMEGA_MAX, "kp_pan": KP_PAN, "ki_pan": KI_PAN,
            "kp_tilt": KP_TILT, "ki_tilt": KI_TILT}


AJUSTES_DEFECTO = ajustes_actuales()


def validar_ajustes(datos):
    """
    Completa `datos` con los valores actuales (se puede enviar solo lo que cambia) y
    devuelve el dict de ajustes validado; lanza ValueError si algo no es válido.
    """
    a = {**ajustes_actuales(), **{k: v for k, v in datos.items() if k in AJUSTES_DEFECTO}}

    if a["objetivo"] not in OBJETIVOS:
        raise ValueError('objetivo debe ser "triangulo" o "laser"')
    if not isinstance(a["ver_mascara"], bool):
        raise ValueError("ver_mascara debe ser true o false")
    for k, (minimo, maximo, decimales) in AJUSTES_NUMERICOS.items():
        try:
            a[k] = float(a[k])
        except (TypeError, ValueError):
            raise ValueError(f"{k} no es un número")
        if not minimo <= a[k] <= maximo:
            raise ValueError(f"{k} debe estar entre {minimo} y {maximo}")
        a[k] = round(a[k], decimales) if decimales else round(a[k])
    return a


def aplicar_ajustes(datos, guardar):
    global OBJETIVO, AREA_MIN, PROPORCION_TOL, AREA_MIN_LASER, CIRCULARIDAD_MIN
    global OMEGA_MAX, KP_PAN, KI_PAN, KP_TILT, KI_TILT
    global TONO_MAX, SAT_MIN, BRILLO_MIN, RELLENO_MIN, EXPOSICION_EV, VER_MASCARA
    a = validar_ajustes(datos)
    TONO_MAX, SAT_MIN, BRILLO_MIN = a["tono_max"], a["sat_min"], a["brillo_min"]
    RELLENO_MIN, EXPOSICION_EV, VER_MASCARA = a["relleno_min"], a["exposicion_ev"], a["ver_mascara"]
    OBJETIVO, AREA_MIN, PROPORCION_TOL = a["objetivo"], a["area_min"], a["proporcion_tol"]
    AREA_MIN_LASER, CIRCULARIDAD_MIN = a["area_min_laser"], a["circularidad_min"]
    OMEGA_MAX, KP_PAN, KI_PAN, KP_TILT, KI_TILT = (a[k] for k in AJUSTES_CONTROL)
    parametros_cambiados.set()  # el bucle de la cámara los envía a la ESP en el próximo frame
    if guardar:
        ARCHIVO_AJUSTES.write_text(json.dumps(a, indent=2))


def cargar_ajustes():
    """Aplica los ajustes guardados desde la página, si existen."""
    if not ARCHIVO_AJUSTES.exists():
        return
    try:
        aplicar_ajustes(json.loads(ARCHIVO_AJUSTES.read_text()), guardar=False)
        print(f"Ajustes cargados de {ARCHIVO_AJUSTES.name}: {ajustes_actuales()}")
    except (OSError, ValueError, AttributeError) as e:
        print(f"No se pudo leer {ARCHIVO_AJUSTES.name} ({e}): se usan los valores del código")


@app.route("/ajustes", methods=["GET", "POST"])
def ajustes():
    """
    GET: ajustes actuales. POST: cambia los que se envíen. La velocidad máxima (omega_max)
    y la vista de la máscara (ver_mascara) se pueden cambiar en cualquier etapa; todos los
    demás, solo durante la etapa Calibrar.
    """
    if request.method == "POST":
        datos = request.get_json(silent=True)
        if not isinstance(datos, dict) or not any(k in AJUSTES_DEFECTO for k in datos):
            return jsonify(error="no se recibió ningún ajuste"), 400
        solo_calibrar = [k for k in datos if k in AJUSTES_DEFECTO and k not in AJUSTES_LIBRES]
        try:
            # Con `lock` tomado la etapa no puede cambiar entre la revisión y el cambio
            with lock:
                if solo_calibrar and etapa != CALIBRAR:
                    return jsonify(error="fuera de la etapa Calibrar solo se pueden cambiar la velocidad máxima y la vista de la máscara"), 409
                aplicar_ajustes(datos, guardar=True)
        except ValueError as e:
            return jsonify(error=str(e)), 400
        except OSError as e:
            return jsonify(error=f"se aplicó, pero no se pudo guardar ({e})"), 500
    return jsonify(**ajustes_actuales(), proporcion=round(PROPORCION, 4), defecto=AJUSTES_DEFECTO)


if __name__ == "__main__":
    cargar_ajustes()
    threading.Thread(target=bucle_camara, daemon=True).start()
    threading.Thread(target=bucle_video, daemon=True).start()
    app.run(host="0.0.0.0", port=PUERTO, threaded=True)
