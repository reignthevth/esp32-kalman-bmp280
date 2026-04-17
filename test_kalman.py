# -*- coding: utf-8 -*-
"""
Kalman filter validation -- mirrors KalmanFilter1D.h logic exactly.

Three test scenarios:
  1. Static sensor  : filtered noise must be much smaller than raw noise
  2. Step change    : filter must track a sudden pressure jump
  3. Slow ramp      : filter must follow a gradual altitude change

Run:
  python test_kalman.py
"""

import math, random, sys

# ---- Kalman filter (mirrors KalmanFilter1D.h exactly) -----------------------
class KalmanFilter1D:
    def __init__(self, Q, R, P0=1.0):
        self.x = 0.0
        self.P = P0
        self.Q = Q
        self.R = R
        self._init = False

    def update(self, z):
        if not self._init:
            self.x = z
            self._init = True
            return self.x
        P_pred = self.P + self.Q
        K      = P_pred / (P_pred + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1.0 - K) * P_pred
        return self.x

    def gain(self):
        return self.P / (self.P + self.R)

# ---- Helpers ----------------------------------------------------------------
def variance(data):
    mu = sum(data) / len(data)
    return sum((x - mu) ** 2 for x in data) / len(data)

def std(data):
    return math.sqrt(variance(data))

def rmse(estimates, truth):
    return math.sqrt(sum((e - t) ** 2 for e, t in zip(estimates, truth)) / len(truth))

def gaussian(sigma):
    u1 = max(random.random(), 1e-12)
    u2 = random.random()
    return sigma * math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)

# ---- Test 1: Static signal --------------------------------------------------
def test_static(Q=0.01, R=1.0, n=500, true_Pa=101325.0, sigma=1.0):
    random.seed(42)
    kf   = KalmanFilter1D(Q, R)
    raws = [true_Pa + gaussian(sigma) for _ in range(n)]
    filt = [kf.update(z) for z in raws]

    raw_std  = std(raws)
    filt_std = std(filt[50:])
    reduction = (1.0 - filt_std / raw_std) * 100.0
    gain      = kf.gain()

    PASS = filt_std < raw_std * 0.5
    status = "PASS" if PASS else "FAIL"
    print("[TEST 1 - Static]  " + status)
    print("  raw  std = %.4f Pa" % raw_std)
    print("  filt std = %.4f Pa  (%.1f%% noise reduction)" % (filt_std, reduction))
    print("  steady-state gain = %.4f  (healthy: 0.05-0.30)" % gain)
    return PASS, raws, filt

# ---- Test 2: Step change ----------------------------------------------------
def test_step(Q=0.01, R=1.0, n=300, Pa_before=101325.0, Pa_after=101200.0, sigma=1.0):
    random.seed(7)
    kf   = KalmanFilter1D(Q, R)
    step = n // 2
    true_vals = [Pa_before if i < step else Pa_after for i in range(n)]
    raws = [t + gaussian(sigma) for t in true_vals]
    filt = [kf.update(z) for z in raws]

    window     = filt[step + 30: step + 60]
    settle_err = abs(sum(window) / len(window) - Pa_after)
    PASS = settle_err < 5.0

    status = "PASS" if PASS else "FAIL"
    print("\n[TEST 2 - Step]    " + status)
    print("  step magnitude = %.0f Pa" % abs(Pa_after - Pa_before))
    print("  error 30 samples after step = %.2f Pa  (threshold: <5 Pa)" % settle_err)
    return PASS, raws, filt, true_vals

# ---- Test 3: Slow ramp ------------------------------------------------------
def test_ramp(Q=0.01, R=1.0, n=400, start=101325.0, rate=-0.5, sigma=1.0):
    """
    Constant-state Kalman model has inherent lag on ramps.
    Theoretical steady-state lag = rate * (1 - K_ss) / K_ss
    where K_ss satisfies: K_ss^2 + (Q/R)*K_ss - Q/R = 0
    With Q=0.01, R=1.0 -> K_ss ~= 0.095 -> lag ~= 4.75 Pa
    This is expected -- to reduce lag, increase Q (at the cost of more noise).
    """
    random.seed(13)
    kf = KalmanFilter1D(Q, R)
    true_vals = [start + rate * i for i in range(n)]
    raws = [t + gaussian(sigma) for t in true_vals]
    filt = [kf.update(z) for z in raws]

    r = rmse(filt[50:], true_vals[50:])

    # Theoretical steady-state lag for this Q/R and rate
    QR = Q / R
    # K_ss from quadratic: K^2 + QR*K - QR = 0
    K_ss = (-QR + math.sqrt(QR * QR + 4 * QR)) / 2.0
    expected_lag = abs(rate) * (1.0 - K_ss) / K_ss

    # PASS if RMSE is within 20% of the theoretical prediction
    PASS = abs(r - expected_lag) < expected_lag * 0.20

    status = "PASS" if PASS else "FAIL"
    print("\n[TEST 3 - Ramp]    " + status)
    print("  rate = %.2f Pa/sample" % rate)
    print("  RMSE(filtered, true) = %.4f Pa" % r)
    print("  theoretical lag      = %.4f Pa  (matches: expected behaviour)" % expected_lag)
    print("  NOTE: increase Q to reduce lag at cost of more noise (Q/R tradeoff)")
    return PASS, raws, filt, true_vals

# ---- Optional plot ----------------------------------------------------------
def try_plot(r1, f1, r2, f2, t2, r3, f3, t3):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed -- skipping plot)")
        return

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    fig.suptitle("KalmanFilter1D Validation  (Q=0.01, R=1.0)", fontweight="bold")

    axes[0].plot(r1, color="#94a3b8", alpha=0.5, lw=0.8, label="raw")
    axes[0].plot(f1, color="#6366f1", lw=1.4, label="filtered")
    axes[0].set_title("Test 1 - Static signal")

    axes[1].plot(r2, color="#94a3b8", alpha=0.5, lw=0.8, label="raw")
    axes[1].plot(f2, color="#6366f1", lw=1.4, label="filtered")
    axes[1].plot(t2, color="#22c55e", lw=1.0, ls="--", label="true")
    axes[1].set_title("Test 2 - Step change")

    axes[2].plot(r3, color="#94a3b8", alpha=0.5, lw=0.8, label="raw")
    axes[2].plot(f3, color="#6366f1", lw=1.4, label="filtered")
    axes[2].plot(t3, color="#22c55e", lw=1.0, ls="--", label="true")
    axes[2].set_title("Test 3 - Slow ramp")

    for ax in axes:
        ax.legend()
        ax.set_ylabel("Pa")
        ax.set_xlabel("sample")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("kalman_validation.png", dpi=150)
    print("\nPlot saved -> kalman_validation.png")
    plt.show()

# ---- Main -------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 52)
    print("  KalmanFilter1D - Validation Suite")
    print("  Q=0.01  R=1.0  (BMP280 defaults)")
    print("=" * 52)

    p1, r1, f1       = test_static()
    p2, r2, f2, t2   = test_step()
    p3, r3, f3, t3   = test_ramp()

    all_pass = p1 and p2 and p3
    print("\n" + "=" * 52)
    print("  Result: " + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
    print("=" * 52)

    try_plot(r1, f1, r2, f2, t2, r3, f3, t3)

    sys.exit(0 if all_pass else 1)
