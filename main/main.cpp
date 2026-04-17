/**
 * main.cpp -- ESP32 BMP280 pressure logger with 1D Kalman filtering.
 *
 * Hardware:  ESP32 + BMP280 (I2C, SDA=GPIO21, SCL=GPIO22)
 * Build:     idf.py build flash monitor
 *
 * Simulation mode (Wokwi CI / no hardware):
 *     SIMULATION_MODE=1 idf.py build
 *   Generates synthetic pressure data, no I2C required.
 *
 * Serial output (CSV, 115200 baud):
 *     t_ms,raw_Pa,filtered_Pa
 */

#include <cinttypes>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

#ifndef SIMULATION_MODE
#include "bmp280.hpp"
#endif

#include "KalmanFilter1D.h"

static const char *TAG            = "main";
static const uint32_t INTERVAL_MS = 50;     // 20 Hz
static const float KALMAN_Q       = 0.01f;
static const float KALMAN_R       = 1.0f;

// Synthetic signal: slow sine wave + uniform ±1 Pa noise
static float synthetic_pressure(float t) {
    float true_pa = 101325.0f + 80.0f * sinf(t * 0.04f);
    float noise   = ((float)(int32_t)(esp_random() % 4001) - 2000) / 2000.0f;
    return true_pa + noise;
}

static void sensor_task(void * /*unused*/) {

#ifdef SIMULATION_MODE
    ESP_LOGI(TAG, "SIMULATION_MODE: synthetic pressure data (no BMP280)");
#else
    BMP280 bmp;
    bool sim_mode = (bmp.begin() != ESP_OK);
    if (sim_mode) {
        ESP_LOGW(TAG, "BMP280 not found -- falling back to simulation");
    }
#endif

    KalmanFilter1D filter(KALMAN_Q, KALMAN_R);
    printf("t_ms,raw_Pa,filtered_Pa\n");

    float sim_t = 0.0f;

    while (true) {
        const int64_t t_ms = esp_timer_get_time() / 1000LL;

#ifdef SIMULATION_MODE
        const float raw = synthetic_pressure(sim_t++);
#else
        const float raw = sim_mode ? synthetic_pressure(sim_t++) : bmp.readPressurePa();
#endif

        const float filt = filter.update(raw);

        if (raw > 0.0f) {
            printf("%" PRId64 ",%.2f,%.2f\n", t_ms, raw, filt);
        }

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Kalman logger -- Q=%.4f R=%.4f @ %lu Hz",
             KALMAN_Q, KALMAN_R, 1000UL / INTERVAL_MS);
    xTaskCreate(sensor_task, "sensor", 4096, nullptr, 5, nullptr);
}
