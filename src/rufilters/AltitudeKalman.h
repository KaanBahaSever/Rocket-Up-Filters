#pragma once

// Vertical-channel Kalman filter: fuses a barometric altimeter and an accelerometer into
// altitude, vertical velocity and vertical acceleration estimates.
//
// State x = [h, v, a] (m, m/s, m/s^2), constant-acceleration model driven by white jerk.
// Measurements are scalar (H = [1 0 0] for altitude, [0 0 1] for acceleration), so no matrix
// inversion is needed: the filter runs comfortably on an 8-bit microcontroller.

#include <math.h>

namespace rufilters {

template <typename T = float>
class AltitudeKalman {
public:
    /// `jerkNoise`: process noise spectral density of the jerk (m^2/s^5). 10-100 for rockets.
    /// `altitudeNoise`: barometer altitude sigma (m). `accelNoise`: accelerometer sigma (m/s^2).
    explicit AltitudeKalman(T jerkNoise = T(50), T altitudeNoise = T(0.5), T accelNoise = T(0.5))
        : q_(jerkNoise), rh_(altitudeNoise * altitudeNoise), ra_(accelNoise * accelNoise) {
        reset(T(0));
    }

    void reset(T altitude, T velocity = T(0), T acceleration = T(0)) {
        x_[0] = altitude;
        x_[1] = velocity;
        x_[2] = acceleration;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) P_[i][j] = T(0);
        P_[0][0] = T(1);
        P_[1][1] = T(1);
        P_[2][2] = T(10);
    }

    void setJerkNoise(T q) { q_ = q; }
    void setAltitudeNoise(T sigma) { rh_ = sigma * sigma; }
    void setAccelNoise(T sigma) { ra_ = sigma * sigma; }

    /// Propagate the state by `dt` seconds.
    void predict(T dt) {
        if (dt <= T(0)) return;
        const T dt2 = dt * dt, dt3 = dt2 * dt;
        // x = F x
        x_[0] += x_[1] * dt + x_[2] * dt2 / T(2);
        x_[1] += x_[2] * dt;
        // P = F P F^T + Q
        const T F[3][3] = {{T(1), dt, dt2 / T(2)}, {T(0), T(1), dt}, {T(0), T(0), T(1)}};
        T FP[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                T s = T(0);
                for (int k = 0; k < 3; ++k) s += F[i][k] * P_[k][j];
                FP[i][j] = s;
            }
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                T s = T(0);
                for (int k = 0; k < 3; ++k) s += FP[i][k] * F[j][k];
                P_[i][j] = s;
            }
        const T dt4 = dt3 * dt, dt5 = dt4 * dt;
        const T Q[3][3] = {{dt5 / T(20), dt4 / T(8), dt3 / T(6)},
                           {dt4 / T(8), dt3 / T(3), dt2 / T(2)},
                           {dt3 / T(6), dt2 / T(2), dt}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) P_[i][j] += q_ * Q[i][j];
    }

    /// Barometric altitude measurement (m).
    void updateAltitude(T altitude) { update(0, altitude, rh_); }
    /// Vertical acceleration measurement (m/s^2, gravity removed, positive up).
    void updateAcceleration(T acceleration) { update(2, acceleration, ra_); }

    T altitude() const { return x_[0]; }
    T velocity() const { return x_[1]; }
    T acceleration() const { return x_[2]; }
    /// Standard deviation of a state component (0 = h, 1 = v, 2 = a).
    T sigma(int i) const { return sqrt(P_[i][i] > T(0) ? P_[i][i] : T(0)); }
    /// Normalised innovation of the last altitude update (|r|/sigma): large values hint at
    /// barometer glitches (transonic pressure spikes, ejection charges).
    T lastAltitudeInnovation() const { return lastInnovation_; }

private:
    T x_[3];
    T P_[3][3];
    T q_, rh_, ra_;
    T lastInnovation_ = T(0);

    void update(int idx, T z, T r) {
        const T y = z - x_[idx];
        const T s = P_[idx][idx] + r;
        if (s <= T(0)) return;
        if (idx == 0) lastInnovation_ = fabs(y) / sqrt(s);
        T K[3];
        for (int i = 0; i < 3; ++i) K[i] = P_[i][idx] / s;
        for (int i = 0; i < 3; ++i) x_[i] += K[i] * y;
        // P = (I - K H) P, H selects row `idx`.
        T row[3];
        for (int j = 0; j < 3; ++j) row[j] = P_[idx][j];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) P_[i][j] -= K[i] * row[j];
        // Keep P symmetric against round-off.
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j) P_[i][j] = P_[j][i] = (P_[i][j] + P_[j][i]) / T(2);
    }
};

}  // namespace rufilters
