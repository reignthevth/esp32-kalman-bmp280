/**
 * main.cpp -- ESP32 BMP280 pressure logger with 1D Kalman filtering.
 *
 * Wiring:  BMP280 VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22
 * Flash:   idf.py build flash monitor
 *
 * Serial output (115200 baud, CSV):
 *     t_ms,raw_Pa,filtered_Pa
 *
 * Capture for MATLAB:
 *     idf.py monitor | findstr /R "^[0-9]" > data.csv
 *
 * Simulation build (no hardware):
 *     SIMULATION_MODE=1 idf.py build
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

static const char    *TAG         = "main";
static const uint32_t INTERVAL_MS = 50;    // 20 Hz
static const float    KALMAN_Q    = 0.01f; // re-tune after R measurement
static const float    KALMAN_R    = 1.0f;  // set to var(raw_Pa) from static recording

#ifdef SIMULATION_MODE
static float synthetic_pressure(float t) {
    float true_pa = 101325.0f + 80.0f * sinf(t * 0.04f);
    float noise   = ((float)(int32_t)(esp_random() % 4001) - 2000) / 2000.0f;
    return true_pa + noise;
}
#endif

static void sensor_task(void *) {
#ifdef SIMULATION_MODE
    ESP_LOGW(TAG, "SIMULATION_MODE -- synthetic data, no hardware needed");
    float sim_t = 0.0f;
#else
    BMP280 bmp;
    if (bmp.begin() != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 not found -- check wiring (SDA=GPIO21, SCL=GPIO22)");
        vTaskDelete(nullptr);
        return;
    }
#endif

    KalmanFilter1D filter(KALMAN_Q, KALMAN_R);
    printf("t_ms,raw_Pa,filtered_Pa\n");

    while (true) {
        const int64_t t_ms = esp_timer_get_time() / 1000LL;

#ifdef SIMULATION_MODE
        const float raw = synthetic_pressure(sim_t++);
#else
        const float raw = bmp.readPressurePa();
#endif
        const float filt = filter.update(raw);

        if (raw > 0.0f)
            printf("%" PRId64 ",%.2f,%.2f\n", t_ms, raw, filt);

        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Kalman logger -- Q=%.4f  R=%.4f  @ %lu Hz",
             KALMAN_Q, KALMAN_R, 1000UL / INTERVAL_MS);
    xTaskCreate(sensor_task, "sensor", 4096, nullptr, 5, nullptr);
}
