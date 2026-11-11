#include "Kalman.h"

#include <math.h>

#define DOS_PI 6.28318531f

void kalmanIniciar(Kalman *k, const ConfigKalman *config)
{
    k->config = *config;
    kalmanReiniciar(k);
}

void kalmanReiniciar(Kalman *k)
{
    k->iniciado = false;
    k->theta = 0.0f;
    k->omega = 0.0f;
    k->p00 = 0.0f;
    k->p01 = 0.0f;
    k->p11 = 0.0f;
}

float kalmanPaso(Kalman *k, float medicion, bool hayMedicion)
{
    const ConfigKalman *c = &k->config;
    float r = c->sigmaMedicion * c->sigmaMedicion;  // varianza de la medición, R

    // La primera medición fija la posición; de la velocidad todavía no se sabe nada
    if (!k->iniciado) {
        if (hayMedicion) {
            k->theta = medicion;
            k->omega = 0.0f;
            k->p00 = r;
            k->p01 = 0.0f;
            k->p11 = c->sigmaOmegaInicial * c->sigmaOmegaInicial;
            k->iniciado = true;
        }
        return k->theta;
    }

    // ---- Predicción: x = F·x,  P = F·P·Fᵀ + Q,  con F = [1 T; 0 1] ----
    float T = c->ts;
    float q = c->sigmaAceleracion * c->sigmaAceleracion;  // varianza de la aceleración

    k->theta += T * k->omega;

    // Q = q·[T⁴/4  T³/2; T³/2  T²]
    k->p00 += T * (2.0f * k->p01 + T * k->p11) + q * T * T * T * T / 4.0f;
    k->p01 += T * k->p11 + q * T * T * T / 2.0f;
    k->p11 += q * T * T;

    // ---- Corrección con la medición: H = [1 0] ----
    if (hayMedicion) {
        // Innovación por el camino más corto: la medición llega entre -π y π
        float y = remainderf(medicion - k->theta, DOS_PI);

        float s = k->p00 + r;  // varianza de la innovación
        float k0 = k->p00 / s;  // ganancia de Kalman, K = P·Hᵀ / s
        float k1 = k->p01 / s;

        k->theta += k0 * y;
        k->omega += k1 * y;

        // P = (I - K·H)·P
        k->p11 -= k1 * k->p01;
        k->p01 -= k0 * k->p01;
        k->p00 -= k0 * k->p00;
    }

    return k->theta;
}
