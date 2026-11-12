#include "encoder_as5600.h"

#include "esp_log.h"
#include "freertos/task.h"

#define AS5600_DIRECCION 0x36
#define AS5600_STATUS 0x0B     // bit 5: imán detectado, bit 4: muy débil, bit 3: muy fuerte
#define AS5600_RAW_ANGLE 0x0C  // 12 bits en 0x0C (alto) y 0x0D (bajo)
#define AS5600_CUENTAS 4096
#define FRECUENCIA_I2C_HZ 400000
#define TIMEOUT_I2C_MS 10
#define PERIODO_LECTURA_MS 10  // entre lecturas el motor debe girar menos de media vuelta
#define DOS_PI 6.28318531f

static const char *TAG = "encoder_as5600";

static esp_err_t leerRegistros(EncoderAs5600 *enc, uint8_t reg, uint8_t *datos, size_t n)
{
    return i2c_master_transmit_receive(enc->dispositivo, &reg, 1, datos, n, TIMEOUT_I2C_MS);
}

// Lee el ángulo crudo y suma la diferencia con la lectura anterior por el camino más corto,
// así el conteo sigue más allá de una vuelta
static bool actualizar(EncoderAs5600 *enc)
{
    uint8_t d[2];
    if (leerRegistros(enc, AS5600_RAW_ANGLE, d, 2) != ESP_OK) {
        taskENTER_CRITICAL(&enc->mux);
        enc->errores++;
        taskEXIT_CRITICAL(&enc->mux);
        return false;
    }

    uint16_t raw = ((d[0] & 0x0F) << 8) | d[1];
    int32_t delta = (int32_t)raw - enc->rawAnterior;
    if (delta > AS5600_CUENTAS / 2) {
        delta -= AS5600_CUENTAS;
    } else if (delta < -AS5600_CUENTAS / 2) {
        delta += AS5600_CUENTAS;
    }
    enc->rawAnterior = raw;

    taskENTER_CRITICAL(&enc->mux);
    enc->cuentas += delta;
    taskEXIT_CRITICAL(&enc->mux);
    return true;
}

static void tareaEncoder(void *arg)
{
    EncoderAs5600 *enc = arg;

    while (true) {
        actualizar(enc);
        vTaskDelay(pdMS_TO_TICKS(PERIODO_LECTURA_MS));
    }
}

void encoderAs5600Iniciar(EncoderAs5600 *enc, const ConfigEncoderAs5600 *config)
{
    enc->config = *config;
    enc->presente = false;
    portMUX_INITIALIZE(&enc->mux);
    enc->rawAnterior = 0;
    enc->cuentas = 0;
    enc->errores = 0;

    i2c_master_bus_config_t bus = {
        .i2c_port = config->puerto,
        .sda_io_num = config->sda,
        .scl_io_num = config->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  // la PCB tiene pull-ups de 4.7 kΩ
    };
    i2c_master_bus_handle_t handleBus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &handleBus));

    if (i2c_master_probe(handleBus, AS5600_DIRECCION, 50) != ESP_OK) {
        ESP_LOGW(TAG, "No responde en SDA %d / SCL %d: queda sin encoder", config->sda, config->scl);
        return;
    }

    i2c_device_config_t dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_DIRECCION,
        .scl_speed_hz = FRECUENCIA_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(handleBus, &dispositivo, &enc->dispositivo));

    uint8_t status;
    if (leerRegistros(enc, AS5600_STATUS, &status, 1) != ESP_OK) {
        ESP_LOGW(TAG, "SDA %d: no se pudo leer el estado del imán", config->sda);
    } else if (!(status & 0x20)) {
        ESP_LOGW(TAG, "SDA %d: no detecta el imán", config->sda);
    } else if (status & 0x18) {
        ESP_LOGW(TAG, "SDA %d: imán muy %s", config->sda, (status & 0x10) ? "débil" : "fuerte");
    }

    // La primera lectura fija el cero en la posición de arranque
    if (!actualizar(enc)) {
        ESP_LOGW(TAG, "SDA %d: falló la primera lectura, queda sin encoder", config->sda);
        return;
    }
    enc->cuentas = 0;
    enc->presente = true;

    xTaskCreate(tareaEncoder, "encoder", 3072, enc, 7, NULL);

    ESP_LOGI(TAG, "Iniciado (SDA %d, SCL %d)", config->sda, config->scl);
}

bool encoderAs5600Leer(EncoderAs5600 *enc, float *angulo)
{
    if (!enc->presente) {
        return false;
    }

    taskENTER_CRITICAL(&enc->mux);
    int32_t cuentas = enc->cuentas;
    taskEXIT_CRITICAL(&enc->mux);

    *angulo = enc->config.signo * cuentas * DOS_PI / AS5600_CUENTAS / enc->config.reduccion;
    return true;
}

void encoderAs5600FijarCero(EncoderAs5600 *enc)
{
    taskENTER_CRITICAL(&enc->mux);
    enc->cuentas = 0;
    taskEXIT_CRITICAL(&enc->mux);
}

uint32_t encoderAs5600Errores(EncoderAs5600 *enc)
{
    taskENTER_CRITICAL(&enc->mux);
    uint32_t errores = enc->errores;
    taskEXIT_CRITICAL(&enc->mux);
    return errores;
}
