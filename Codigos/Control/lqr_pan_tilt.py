#!/usr/bin/env python3
"""
Diseño y simulación de control LQR (con acción integral) + filtro de Kalman
para la plataforma pan-tilt 2-DOF.

Modelo por eje (continuo):
    dθ/dt  = ω
    dω/dt  = -(1/τ)·ω + (K/τ)·u
    dθo/dt = ωo
    dωo/dt = w            (objetivo a velocidad casi constante)

Estado del LQR (coordenadas de error, con prealimentación u_ff = ωo/K):
    z = [ e, ω̃, ξ ]   con  e = θo - θ,  ω̃ = ω - ωo,  ξ = ∫e dt
    u = ωo/K - Klqr·z

El Kalman estima x = [θ, ω, θo, ωo] usando el encoder (siempre) y la cámara
(solo cuando hay detección). Si se pierde el objetivo, θo se sigue prediciendo.

Uso:
    python lqr_pan_tilt.py          (imprime ganancias y grafica la simulación)
"""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

# ---------------- Parámetros ----------------
TS = 1 / 30            # período de control = período de la cámara [s]
ANCHO, ALTO = 640, 480
# Camera Module 2, modo del sensor 1640x1232 (sensor completo) escalado a 640x480:
# FOV 62.2° x 48.8°  ->  focal en píxeles (~530). Con el modo nativo 640x480
# (recorte) la focal sería ~1357 px.
F_X = (ANCHO / 2) / np.tan(np.radians(62.2 / 2))
F_Y = (ALTO / 2) / np.tan(np.radians(48.8 / 2))


@dataclass
class Eje:
    nombre: str
    K: float      # ganancia del motor [rad/s por V]  (identificar con escalón)
    tau: float    # constante de tiempo mecánica [s]
    v_max: float  # saturación del actuador [V]
    f: float      # focal en píxeles para este eje


PAN = Eje("pan", K=0.9, tau=0.05, v_max=12.0, f=F_X)
TILT = Eje("tilt", K=0.9, tau=0.06, v_max=12.0, f=F_Y)

# Bryson: 1 / (valor máximo aceptable)^2
E_MAX = np.radians(0.5)    # error de apuntamiento aceptable [rad]
W_MAX = 2.0                # error de velocidad aceptable [rad/s]
XI_MAX = np.radians(0.5)   # error integral aceptable [rad·s]

# Ruidos para el Kalman
SIGMA_PX = 2.0                     # ruido de detección [px]
SIGMA_ENC = np.radians(0.1)        # cuantización del encoder [rad]
SIGMA_ACC_OBJ = 3.0                # aceleración típica del objetivo [rad/s^2]


# ---------------- Diseño ----------------
def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def disenar_lqr(eje: Eje):
    """LQI discreto sobre z = [e, ω̃, ξ]."""
    Ac = np.array([[0, -1, 0],
                   [0, -1 / eje.tau, 0],
                   [1, 0, 0]])
    Bc = np.array([[0], [eje.K / eje.tau], [0]])
    Ad, Bd, *_ = cont2discrete((Ac, Bc, np.eye(3), np.zeros((3, 1))), TS)
    Q = np.diag([1 / E_MAX**2, 1 / W_MAX**2, 1 / XI_MAX**2])
    R = np.array([[1 / eje.v_max**2]])
    Klqr = dlqr(Ad, Bd, Q, R)
    polos = np.linalg.eigvals(Ad - Bd @ Klqr)
    return Klqr, polos


def modelo_kalman(eje: Eje):
    """Modelo discreto de 4 estados x = [θ, ω, θo, ωo]."""
    a = np.exp(-TS / eje.tau)
    A = np.array([[1, eje.tau * (1 - a), 0, 0],
                  [0, a, 0, 0],
                  [0, 0, 1, TS],
                  [0, 0, 0, 1]])
    B = np.array([eje.K * (TS - eje.tau * (1 - a)), eje.K * (1 - a), 0, 0])
    # Ruido de aceleración del objetivo (modelo de velocidad constante)
    G = np.array([0, 0, TS**2 / 2, TS])
    Q = np.outer(G, G) * SIGMA_ACC_OBJ**2 + np.diag([1e-8, 1e-4, 0, 0])
    # Filas de medición: encoder θ y cámara e = θo - θ (ya convertida a rad)
    C = np.array([[1, 0, 0, 0],
                  [-1, 0, 1, 0]])
    R = np.diag([SIGMA_ENC**2, (SIGMA_PX / eje.f)**2])
    return A, B, Q, C, R


class ControladorEje:
    """Kalman + LQI para un eje. Llamar a paso() una vez por período TS."""

    def __init__(self, eje: Eje):
        self.eje = eje
        self.Klqr, self.polos = disenar_lqr(eje)
        self.A, self.B, self.Q, self.C, self.R = modelo_kalman(eje)
        self.x = np.zeros(4)
        self.P = np.diag([1e-4, 1e-2, 0.1, 1.0])
        self.xi = 0.0
        self.u = 0.0

    def paso(self, theta_enc, error_cam=None):
        """
        theta_enc : ángulo medido por el encoder [rad]
        error_cam : error del objetivo respecto al eje óptico [rad], o None si
                    no hay detección (oclusión / pérdida)
        Devuelve el voltaje a aplicar [V].
        """
        # Predicción con la u aplicada en el paso anterior
        self.x = self.A @ self.x + self.B * self.u
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Corrección: encoder siempre, cámara solo si hay detección
        if error_cam is None:
            C, R, y = self.C[:1], self.R[:1, :1], np.array([theta_enc])
        else:
            C, R, y = self.C, self.R, np.array([theta_enc, error_cam])
        S = C @ self.P @ C.T + R
        L = self.P @ C.T @ np.linalg.inv(S)
        self.x = self.x + L @ (y - C @ self.x)
        self.P = (np.eye(4) - L @ C) @ self.P

        theta, omega, theta_o, omega_o = self.x
        e = theta_o - theta
        # El integrador solo acumula con el objetivo visible (evita windup)
        if error_cam is not None:
            self.xi += TS * e

        z = np.array([e, omega - omega_o, self.xi])
        u = omega_o / self.eje.K - (self.Klqr @ z).item()
        u_sat = float(np.clip(u, -self.eje.v_max, self.eje.v_max))
        # Anti-windup: si satura, deshacer la integración de este paso
        if u_sat != u and error_cam is not None:
            self.xi -= TS * e
        self.u = u_sat
        return u_sat


# ---------------- Conversión píxeles -> error angular ----------------
def errores_desde_pixeles(x_px, y_px, theta_tilt):
    """
    Convierte la detección (x, y) en píxeles a (e_pan, e_tilt) en rad.
    Convención: pan positivo = girar a la izquierda, tilt positivo = arriba.
    Si en tu montaje el motor gira al revés, invierte el signo correspondiente.
    """
    dx = x_px - ANCHO / 2          # + hacia la derecha
    dy = y_px - ALTO / 2           # + hacia abajo (convención de imagen)
    e_pan = -np.arctan(dx / F_X) / max(np.cos(theta_tilt), 0.2)
    e_tilt = -np.arctan(dy / F_Y)
    return e_pan, e_tilt


# ---------------- Modelo exacto (convención OpenCV) ----------------
# Base:   X adelante, Y izquierda, Z arriba
# Cámara: x derecha,  y abajo,     z eje óptico  (igual que cv2.calibrateCamera)
CX, CY = ANCHO / 2, ALTO / 2     # centro óptico (reemplazar por el de la calibración)


def rotacion_base_camara(theta_pan, theta_tilt):
    """
    R_bc = Rz(θpan)·Ry(-θtilt)·M. Lleva vectores del marco cámara al marco base.
    Columnas: ejes x (derecha), y (abajo) y z (óptico) de la cámara vistos desde la base.
    """
    cp, sp = np.cos(theta_pan), np.sin(theta_pan)
    ct, st = np.cos(theta_tilt), np.sin(theta_tilt)
    return np.array([[sp, cp * st, cp * ct],
                     [-cp, sp * st, sp * ct],
                     [0.0, -ct, st]])


def proyectar(p_base, theta_pan, theta_tilt):
    """Punto 3D en el marco base -> píxel (u, v). Supone r = 0 (cámara en el cruce de ejes)."""
    p_c = rotacion_base_camara(theta_pan, theta_tilt).T @ np.asarray(p_base, float)
    return F_X * p_c[0] / p_c[2] + CX, F_Y * p_c[1] / p_c[2] + CY


def angulos_objetivo(x_px, y_px, theta_pan, theta_tilt):
    """
    Dirección absoluta del objetivo respecto de la base (azimut, elevación) [rad].
    Versión exacta de θobjetivo = θplataforma + error, válida para ángulos grandes.
    """
    rayo_c = np.array([(x_px - CX) / F_X, (y_px - CY) / F_Y, 1.0])
    rayo_b = rotacion_base_camara(theta_pan, theta_tilt) @ rayo_c
    azimut = np.arctan2(rayo_b[1], rayo_b[0])
    elevacion = np.arctan2(rayo_b[2], np.hypot(rayo_b[0], rayo_b[1]))
    return azimut, elevacion


# ---------------- Simulación ----------------
def simular(eje: Eje, objetivo, t_final=10.0, oclusion=(4.0, 5.0), semilla=0):
    rng = np.random.default_rng(semilla)
    ctrl = ControladorEje(eje)
    sub = 20                           # subpasos para integrar la planta real
    dt = TS / sub
    theta, omega = 0.0, 0.0
    n = int(t_final / TS)
    log = {k: np.zeros(n) for k in ("t", "theta", "theta_o", "theta_o_est", "u", "visible")}

    for k in range(n):
        t = k * TS
        theta_o = objetivo(t)
        visible = not (oclusion[0] <= t < oclusion[1])

        theta_enc = np.round(theta / SIGMA_ENC) * SIGMA_ENC
        error_cam = None
        if visible:
            px = eje.f * np.tan(theta_o - theta) + rng.normal(0, SIGMA_PX)
            error_cam = np.arctan(px / eje.f)

        u = ctrl.paso(theta_enc, error_cam)

        for _ in range(sub):           # planta "real" en tiempo continuo
            omega += dt * (-omega + eje.K * u) / eje.tau
            theta += dt * omega

        log["t"][k], log["theta"][k], log["theta_o"][k] = t, theta, theta_o
        log["theta_o_est"][k], log["u"][k], log["visible"][k] = ctrl.x[2], u, visible
    return ctrl, log


def objetivo_pan(t):
    """Movimiento 'aleatorio' suave + salto a t=7 s."""
    return np.radians(20 * np.sin(0.6 * t) + 8 * np.sin(1.7 * t + 1) + (10 if t > 7 else 0))


def objetivo_tilt(t):
    return np.radians(10 * np.sin(0.4 * t + 0.5) + 4 * np.sin(1.3 * t))


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    print(f"TS = {TS*1000:.1f} ms,  f_x = {F_X:.1f} px,  f_y = {F_Y:.1f} px\n")
    fig, ax = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for col, (eje, obj) in enumerate([(PAN, objetivo_pan), (TILT, objetivo_tilt)]):
        ctrl, log = simular(eje, obj)
        print(f"[{eje.nombre}]  Klqr = {np.array2string(ctrl.Klqr.ravel(), precision=2)}"
              f"   |polos| = {np.array2string(np.abs(ctrl.polos), precision=3)}")
        err = np.degrees(log["theta_o"] - log["theta"])
        vis = log["visible"].astype(bool)
        print(f"         error RMS visible = {np.sqrt(np.mean(err[vis][30:]**2)):.3f}°"
              f"   error máx en oclusión = {np.max(np.abs(err[~vis])):.2f}°")

        t = log["t"]
        ax[0, col].plot(t, np.degrees(log["theta_o"]), label="objetivo")
        ax[0, col].plot(t, np.degrees(log["theta"]), label="plataforma")
        ax[0, col].plot(t, np.degrees(log["theta_o_est"]), "--", label="objetivo (Kalman)")
        ax[0, col].set_ylabel("ángulo [°]")
        ax[0, col].set_title(eje.nombre)
        ax[0, col].legend()
        ax[1, col].plot(t, err)
        ax[1, col].set_ylabel("error [°]")
        ax[2, col].plot(t, log["u"])
        ax[2, col].set_ylabel("u [V]")
        ax[2, col].set_xlabel("t [s]")
        for a in ax[:, col]:
            a.axvspan(4, 5, color="gray", alpha=0.2)
            a.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()
