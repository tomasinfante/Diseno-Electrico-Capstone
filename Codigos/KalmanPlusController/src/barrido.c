#include "barrido.h"

#include <math.h>

#define DOS_PI 6.28318531f

void barridoIniciar(Barrido *b, const ConfigBarrido *config)
{
    b->config = *config;
    barridoReiniciar(b, 0.0f);
}

void barridoReiniciar(Barrido *b, float angulo)
{
    // Fase en que la senoidal pasa por la posición actual (o por el extremo más cercano)
    float x = angulo / b->config.amplitud;
    b->fase = asinf(fminf(fmaxf(x, -1.0f), 1.0f));
    b->t = 0.0f;
}

float barridoPaso(Barrido *b, float angulo, float omegaMax)
{
    const ConfigBarrido *c = &b->config;
    float w = DOS_PI / c->periodo;  // ritmo normal de la fase [rad/s]

    // Ritmo de la fase respecto del normal (0 a 1): crece durante el arranque y se reduce
    // si la velocidad máxima no alcanza para la velocidad de punta de la senoidal
    float arranque = fminf(b->t / c->tArranque, 1.0f);
    float ritmo = arranque;
    float omegaPunta = c->amplitud * w;
    if (omegaPunta > omegaMax) {
        ritmo *= omegaMax / omegaPunta;
    }

    b->t += c->ts;
    b->fase += w * ritmo * c->ts;
    if (b->fase > DOS_PI) {
        b->fase -= DOS_PI;
    }

    float ref = c->amplitud * sinf(b->fase);
    float omegaRef = c->amplitud * cosf(b->fase) * w * ritmo;

    // El límite de velocidad también crece durante el arranque: si el eje parte lejos de la
    // referencia (fuera del rango del barrido), la corrección no le pide un salto de velocidad
    float limite = omegaMax * arranque;
    float omega = omegaRef + c->Kp * (ref - angulo);
    return fminf(fmaxf(omega, -limite), limite);
}
