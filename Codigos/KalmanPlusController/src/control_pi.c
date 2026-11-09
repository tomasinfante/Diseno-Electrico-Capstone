#include "control_pi.h"

#include <math.h>

void controlPiIniciar(ControlPi *c, const ConfigControlPi *config)
{
    c->config = *config;
    c->theta = 0.0f;
    c->xi = 0.0f;
    c->tPerdido = 0.0f;
}

float controlPiPaso(ControlPi *c, float e, bool visible)
{
    const ConfigControlPi *p = &c->config;

    if (visible) {
        c->xi += p->ts * e;  // el integrador solo acumula con el objetivo visible
        c->tPerdido = 0.0f;
    } else {
        e = 0.0f;  // sin medición: el integrador mantiene la última velocidad de seguimiento
        c->tPerdido += p->ts;
    }

    float omega = 0.0f;
    if (visible || c->tPerdido <= p->tPrediccion) {
        omega = p->Kp * e + p->Ki * c->xi;
    } else {
        c->xi = 0.0f;  // objetivo perdido: se detiene y se olvida la integral
    }

    float omegaSat = fminf(fmaxf(omega, -p->omegaMax), p->omegaMax);
    if (omegaSat != omega && visible) {
        c->xi -= p->ts * e;  // anti-windup: si satura, se deshace la integración de este paso
    }

    c->theta += omegaSat * p->ts;
    return omegaSat;
}
