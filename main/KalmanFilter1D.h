/**
 * KalmanFilter1D.h
 *
 * Scalar (1-dimensional) Kalman filter — framework-agnostic C++, works on
 * both Arduino and ESP-IDF.
 *
 * Model
 * -----
 *   State:       x ~ N(x_hat, P)
 *   Process:     x_k = x_{k-1} + w,   w ~ N(0, Q)   (constant-state assumption)
 *   Measurement: z_k = x_k   + v,     v ~ N(0, R)
 *
 * Recursive update (why it's cheap on a microcontroller — no history needed):
 *   predict:  P_pred = P + Q
 *   update:   K      = P_pred / (P_pred + R)    // Kalman gain in (0, 1)
 *             x      = x + K * (z - x)          // weighted blend
 *             P      = (1 - K) * P_pred         // posterior variance shrinks
 *
 * Tuning
 * ------
 *   R  — measure experimentally: hold sensor still 60 s, compute var(raw_Pa).
 *   Q  — start at 0.01; increase if output lags real motion, decrease if noisy.
 *   Only the ratio Q/R governs steady-state behaviour (Kalman gain).
 */

#pragma once

class KalmanFilter1D {
public:
    /**
     * @param Q   Process noise variance (how much the true state can drift
     *            between two consecutive samples).
     * @param R   Measurement noise variance (sensor intrinsic noise).
     * @param P0  Initial estimate variance — large value makes the filter
     *            trust early measurements and converge quickly.
     */
    KalmanFilter1D(float Q, float R, float P0 = 1.0f)
        : x_(0.0f), P_(P0), Q_(Q), R_(R), initialized_(false) {}

    /**
     * Feed one measurement, return the filtered estimate.
     * First call seeds the state directly (avoids cold-start convergence lag).
     */
    float update(float z) {
        if (!initialized_) {
            x_ = z;
            initialized_ = true;
            return x_;
        }
        // Prediction step
        const float P_pred = P_ + Q_;
        // Update step
        const float K = P_pred / (P_pred + R_);
        x_ = x_ + K * (z - x_);
        P_ = (1.0f - K) * P_pred;
        return x_;
    }

    float getEstimate() const { return x_; }
    float getVariance() const { return P_; }
    // Steady-state gain: healthy range 0.05 – 0.3. Near 1 = filter does nothing.
    float getGain()     const { return P_ / (P_ + R_); }

    void setQ(float Q) { Q_ = Q; }
    void setR(float R) { R_ = R; }
    void reset()       { initialized_ = false; P_ = 1.0f; }

private:
    float x_;
    float P_;
    float Q_;
    float R_;
    bool  initialized_;
};
