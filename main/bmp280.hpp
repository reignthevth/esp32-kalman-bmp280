/**
 * bmp280.hpp — Minimal ESP-IDF I2C driver for the BMP280 barometric sensor.
 *
 * Uses the legacy i2c_master_write_to_device / i2c_master_write_read_device
 * helpers (ESP-IDF 4.3+, still available in 5.x).
 *
 * Hardware IIR filter is disabled by default — the software Kalman filter
 * handles noise. Stacking two filters distorts the signal and makes tuning
 * non-intuitive.
 *
 * Pressure compensation follows the integer formulas from the Bosch BMP280
 * datasheet (section 4.2.3). Temperature is computed only because t_fine
 * is an internal dependency of the pressure formula.
 */

#pragma once

#include <cstdint>
#include <cinttypes>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *BMP280_TAG = "BMP280";

// ---- Register map -------------------------------------------------------
#define BMP280_REG_CALIB_START  0x88u
#define BMP280_REG_CHIP_ID      0xD0u
#define BMP280_REG_CTRL_MEAS    0xF4u
#define BMP280_REG_CONFIG       0xF5u
#define BMP280_REG_PRESS_MSB    0xF7u

#define BMP280_CHIP_ID_VALUE    0x60u

class BMP280 {
public:
    struct Config {
        uint8_t    i2c_addr    = 0x76;
        i2c_port_t i2c_port    = I2C_NUM_0;
        gpio_num_t sda_pin     = GPIO_NUM_21;
        gpio_num_t scl_pin     = GPIO_NUM_22;
        uint32_t   clk_hz      = 400000;

        // oversampling: 0=skip, 1=×1, 2=×2, 3=×4, 4=×8, 5=×16
        uint8_t osrs_t = 2;   // temperature ×2
        uint8_t osrs_p = 5;   // pressure ×16 (best resolution, ~0.16 Pa RMS)

        // IIR filter coefficient: 0=OFF, 1=2, 2=4, 3=8, 4=16
        // Keep OFF so the Kalman filter sees raw sensor noise.
        uint8_t iir = 0;

        // Standby time between measurements (normal mode):
        // 0=0.5 ms, 1=62.5 ms, 2=125 ms, ...
        uint8_t t_sb = 0;  // 0.5 ms → fastest continuous output
    };

    explicit BMP280(const Config &cfg = Config{}) : cfg_(cfg) {}

    esp_err_t begin() {
        // Configure and install I2C master driver
        i2c_config_t ic{};
        ic.mode             = I2C_MODE_MASTER;
        ic.sda_io_num       = cfg_.sda_pin;
        ic.scl_io_num       = cfg_.scl_pin;
        ic.sda_pullup_en    = GPIO_PULLUP_ENABLE;
        ic.scl_pullup_en    = GPIO_PULLUP_ENABLE;
        ic.master.clk_speed = cfg_.clk_hz;
        ESP_RETURN_ON_ERROR(i2c_param_config(cfg_.i2c_port, &ic),
                            BMP280_TAG, "i2c_param_config");
        ESP_RETURN_ON_ERROR(
            i2c_driver_install(cfg_.i2c_port, I2C_MODE_MASTER, 0, 0, 0),
            BMP280_TAG, "i2c_driver_install");

        // Verify chip ID
        uint8_t chip_id = 0;
        ESP_RETURN_ON_ERROR(read_reg(BMP280_REG_CHIP_ID, &chip_id, 1),
                            BMP280_TAG, "read chip_id");
        if (chip_id != BMP280_CHIP_ID_VALUE) {
            ESP_LOGE(BMP280_TAG, "wrong chip ID: 0x%02X (expected 0x60)", chip_id);
            return ESP_ERR_NOT_FOUND;
        }

        // Read factory calibration trimming parameters
        ESP_RETURN_ON_ERROR(load_calibration(), BMP280_TAG, "load_calibration");

        // config register:  t_sb[7:5] | filter[4:2] | 0[1] | spi3w_en[0]
        uint8_t config_val = (uint8_t)((cfg_.t_sb << 5) | (cfg_.iir << 2));
        // ctrl_meas:  osrs_t[7:5] | osrs_p[4:2] | mode[1:0]  — mode=3: normal
        uint8_t ctrl_val   = (uint8_t)((cfg_.osrs_t << 5) | (cfg_.osrs_p << 2) | 0x03);

        ESP_RETURN_ON_ERROR(write_reg(BMP280_REG_CONFIG,    config_val), BMP280_TAG, "write config");
        ESP_RETURN_ON_ERROR(write_reg(BMP280_REG_CTRL_MEAS, ctrl_val),   BMP280_TAG, "write ctrl_meas");

        ESP_LOGI(BMP280_TAG, "OK — osrs_p=×%d, IIR=%s",
                 1 << (cfg_.osrs_p - 1),
                 cfg_.iir == 0 ? "OFF (intentional)" : "ON");
        return ESP_OK;
    }

    /**
     * Read compensated pressure in Pascal (returns -1.0f on I2C error).
     * Burst-reads all 6 bytes (press + temp) in one transaction.
     */
    float readPressurePa() {
        uint8_t buf[6];
        if (read_reg(BMP280_REG_PRESS_MSB, buf, 6) != ESP_OK) {
            ESP_LOGE(BMP280_TAG, "readPressurePa: I2C read failed");
            return -1.0f;
        }
        int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
        int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

        int32_t t_fine = compute_t_fine(adc_T);
        return (float)compensate_pressure(adc_P, t_fine);
    }

private:
    Config cfg_;

    // Bosch trimming parameters (BMP280 datasheet §4.2.2)
    struct Calib {
        uint16_t T1;
        int16_t  T2, T3;
        uint16_t P1;
        int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
    } cal_{};

    // ---- I2C helpers --------------------------------------------------------
    esp_err_t write_reg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        return i2c_master_write_to_device(cfg_.i2c_port, cfg_.i2c_addr,
                                          buf, 2, pdMS_TO_TICKS(100));
    }

    esp_err_t read_reg(uint8_t reg, uint8_t *out, size_t len) {
        return i2c_master_write_read_device(cfg_.i2c_port, cfg_.i2c_addr,
                                            &reg, 1, out, len, pdMS_TO_TICKS(100));
    }

    // ---- Calibration --------------------------------------------------------
    esp_err_t load_calibration() {
        uint8_t b[24];
        ESP_RETURN_ON_ERROR(read_reg(BMP280_REG_CALIB_START, b, 24),
                            BMP280_TAG, "read calib bytes");

        // Little-endian 16-bit reads
        auto u16 = [&](int i) -> uint16_t { return (uint16_t)(b[i] | (b[i+1] << 8)); };
        auto s16 = [&](int i) -> int16_t  { return (int16_t)u16(i); };

        cal_.T1 = u16(0);  cal_.T2 = s16(2);  cal_.T3 = s16(4);
        cal_.P1 = u16(6);  cal_.P2 = s16(8);  cal_.P3 = s16(10);
        cal_.P4 = s16(12); cal_.P5 = s16(14); cal_.P6 = s16(16);
        cal_.P7 = s16(18); cal_.P8 = s16(20); cal_.P9 = s16(22);
        return ESP_OK;
    }

    // ---- Bosch compensation formulas (BMP280 datasheet §4.2.3) --------------
    int32_t compute_t_fine(int32_t adc_T) {
        int32_t v1 = ((((adc_T >> 3) - ((int32_t)cal_.T1 << 1))) * (int32_t)cal_.T2) >> 11;
        int32_t v2 = (((((adc_T >> 4) - (int32_t)cal_.T1) *
                         ((adc_T >> 4) - (int32_t)cal_.T1)) >> 12) *
                       (int32_t)cal_.T3) >> 14;
        return v1 + v2;
    }

    double compensate_pressure(int32_t adc_P, int32_t t_fine) {
        int64_t v1 = (int64_t)t_fine - 128000;
        int64_t v2 = v1 * v1 * (int64_t)cal_.P6;
        v2 += (v1 * (int64_t)cal_.P5) << 17;
        v2 += (int64_t)cal_.P4 << 35;
        v1  = ((v1 * v1 * (int64_t)cal_.P3) >> 8) + ((v1 * (int64_t)cal_.P2) << 12);
        v1  = ((INT64_C(1) << 47) + v1) * (int64_t)cal_.P1 >> 33;
        if (v1 == 0) return 0.0;   // division-by-zero guard
        int64_t p = 1048576 - adc_P;
        p = ((p << 31) - v2) * 3125 / v1;
        v1 = ((int64_t)cal_.P9 * (p >> 13) * (p >> 13)) >> 25;
        v2 = ((int64_t)cal_.P8 * p) >> 19;
        p  = ((p + v1 + v2) >> 8) + ((int64_t)cal_.P7 << 4);
        return (double)p / 256.0;  // Pa
    }
};
