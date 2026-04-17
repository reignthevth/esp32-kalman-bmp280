/**
 * main.cpp — ESP32 barometric pressure logger with 1D Kalman filtering.
 *
 * Hardware:  ESP32 + BMP280 (I2C, SDA=GPIO21, SCL=GPIO22)
 * Build:     idf.py build flash monitor
 *
 * Serial output (115200 baud, CSV):
 *     t_ms,raw_Pa,filtered_Pa
 *
 * Capture to file for MATLAB analysis:
 *     idf.py monitor | grep "^[0-9]" > data.csv
 * Then in MATLAB:
 *     T = readtable('data.csv');
 *     plot(T.t_ms, T.raw_Pa, T.t_ms, T.filtered_Pa);
 *     legend('raw','filtered');
 *
 * Tuning Q and R after recording:
 *     R = var(T.raw_Pa(1:200))   % static 10-second window
 *     Then adjust Q until filtered output tracks real pressure changes
 *     without excessive lag or residual noise.
 */

#include <cinttypes>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bmp280.hpp"
#include "KalmanFilter1D.h"

static const char *TAG = "main";

// Sampling rate: 20 Hz (50 ms between measurements)
static const uint32_t SAMPLE_INTERVAL_MS = 50;

// Initial filter parameters — re-tune after MATLAB analysis:
//   R = var(raw_Pa) from a static recording (sensor held still)
//   Q: increase if filtered output lags real motion, decrease if still noisy
static const float KALMAN_Q = 0.01f;
static const float KALMAN_R = 1.0f;

static void sensor_task(void * /*unused*/) {
    BMP280 bmp;

    if (bmp.begin() != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialisation failed — check wiring (SDA=21, SCL=22)");
        vTaskDelete(nullptr);
        return;
    }

    KalmanFilter1D filter(KALMAN_Q, KALMAN_R);

    // CSV header — parseable by MATLAB readtable() and Python pandas.read_csv()
    printf("t_ms,raw_Pa,filtered_Pa\n");

    while (true) {
        const int64_t t_ms   = esp_timer_get_time() / 1000LL;
        const float   raw    = bmp.readPressurePa();
        const float   filtered = filter.update(raw);

        if (raw > 0.0f) {
            // Use printf (not ESP_LOGI) so the CSV is clean — no log-level prefixes.
            printf("%" PRId64 ",%.2f,%.2f\n", t_ms, raw, filtered);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Barometer Kalman logger — Q=%.4f R=%.4f @ %lu Hz",
             KALMAN_Q, KALMAN_R, 1000UL / SAMPLE_INTERVAL_MS);

    xTaskCreate(
        sensor_task,
        "sensor",
        4096,       // stack: 4 kB is sufficient for I2C + printf
        nullptr,
        5,          // priority 5 (above idle, below most system tasks)
        nullptr
    );
}
