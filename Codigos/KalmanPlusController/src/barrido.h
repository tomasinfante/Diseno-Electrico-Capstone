/*
 * Barrido suave y periódico de un eje, para buscar al objetivo.
 *
 * La referencia es una senoidal alrededor del cero del eje:
 *     θref = amplitud · sin(φ),   con φ avanzando 2π cada `periodo`
 * así el eje recorre un rango de 2·amplitud y frena suavemente en los extremos. La velocidad
 * que se le pide al motor es la de la referencia más una corrección proporcional al error
 * de posición, saturada a la velocidad máxima:
 *     ω = dθref/dt + Kp·(θref - θ)
 *
 * Para que el movimiento sea suave:
 *     - parte desde el punto de la senoidal más cercano a la posición actual del eje;
 *     - el avance de φ y el límite de velocidad crecen de a poco durante `tArranque`, así la
 *       velocidad parte de cero aunque el eje esté fuera del rango del barrido;
 *     - si la velocidad máxima no alcanza para el período pedido, φ avanza más lento:
 *       se recorre el mismo rango en más tiempo.
 *
 * Uso (una instancia por eje):
 *     static Barrido barrido;
 *     barridoIniciar(&barrido, &config);
 *     barridoReiniciar(&barrido, anguloActual);                       // al empezar a buscar
 *     float omega = barridoPaso(&barrido, anguloActual, omegaMax);    // una vez cada config.ts
 */

#pragma once

typedef struct {
    float amplitud;   // el eje se mueve entre -amplitud y +amplitud [rad]
    float periodo;    // duración de una ida y vuelta completa [s]
    float Kp;         // corrección del error de posición [1/s]
    float tArranque;  // tiempo en que el barrido llega a su ritmo normal [s]
    float ts;         // período con que se llama a barridoPaso [s]
} ConfigBarrido;

typedef struct {
    ConfigBarrido config;
    float fase;  // φ [rad]
    float t;     // tiempo desde que empezó el barrido [s]
} Barrido;

void barridoIniciar(Barrido *b, const ConfigBarrido *config);

// Empieza un barrido nuevo desde el punto de la senoidal más cercano a `angulo` [rad]
void barridoReiniciar(Barrido *b, float angulo);

// Un paso del barrido. `angulo` es la posición actual del eje [rad] y `omegaMax` la velocidad
// máxima permitida [rad/s]. Devuelve la velocidad [rad/s] que debe tener el eje hasta el próximo paso.
float barridoPaso(Barrido *b, float angulo, float omegaMax);
