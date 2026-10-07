#pragma once

// Small building blocks: barometric altitude and first-order low-pass filter.

#include <math.h>

namespace rufilters {

/// Altitude (m) above the reference pressure `p0` from the standard-atmosphere troposphere
/// formula h = 44330.77 (1 - (p/p0)^0.190263). Valid up to 11 km on Earth.
template <typename T = float>
inline T pressureToAltitude(T pressure, T p0) {
    if (pressure <= T(0) || p0 <= T(0)) return T(0);
    return T(44330.77) * (T(1) - pow(pressure / p0, T(0.190263)));
}

/// First-order low-pass filter with time constant `tau` (s); handles irregular sampling.
template <typename T = float>
class LowPass {
public:
    explicit LowPass(T tau = T(0.1)) : tau_(tau) {}
    T update(T value, T dt) {
        if (!init_) {
            y_ = value;
            init_ = true;
            return y_;
        }
        const T a = dt / (tau_ + dt);
        y_ += a * (value - y_);
        return y_;
    }
    T value() const { return y_; }
    void reset() { init_ = false; }

private:
    T tau_;
    T y_ = T(0);
    bool init_ = false;
};

/// Running average of the first `n` samples (e.g. ground pressure before launch).
template <typename T = float>
class Calibrator {
public:
    explicit Calibrator(int samples = 50) : n_(samples) {}
    /// Returns true once enough samples were collected.
    bool add(T value) {
        if (count_ < n_) {
            sum_ += value;
            ++count_;
        }
        return done();
    }
    bool done() const { return count_ >= n_; }
    T mean() const { return count_ ? sum_ / T(count_) : T(0); }

private:
    int n_;
    int count_ = 0;
    T sum_ = T(0);
};

}  // namespace rufilters
