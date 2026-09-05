#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36

#define SDA_PIN 21
#define SCL_PIN 22

const float OFFSET_ANGULO = 138.076f;

uint16_t leerAnguloRaw() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0C);

    if (Wire.endTransmission(false) != 0) {
        return 0;
    }

    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte = Wire.read();

        return ((highByte & 0x0F) << 8) | lowByte;
    }

    return 0;
}

float corregirAngulo(float angulo) {

    float corregido = angulo - OFFSET_ANGULO;

    if (corregido < 0.0f) {
        corregido += 360.0f;
    }

    return corregido;
}

void setup() {

    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    delay(500);

    Serial.println("AS5600 iniciado");
}

void loop() {

    uint16_t raw = leerAnguloRaw();

    float anguloOriginal =
        raw * 360.0f / 4096.0f;

    float anguloCorregido =
        corregirAngulo(anguloOriginal);

    Serial.print("Original: ");
    Serial.print(anguloOriginal, 3);

    Serial.print(" | Corregido: ");
    Serial.print(anguloCorregido, 3);

    Serial.println(" grados");

    delay(100);
}