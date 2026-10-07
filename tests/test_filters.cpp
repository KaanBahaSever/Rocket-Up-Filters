// Tests on synthetic flights with sensor noise. Built as C++11 to guarantee the headers stay
// usable with older embedded toolchains.

#include <RocketUpFilters.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

int failures = 0, checks = 0;
#define CHECK(cond, ...)                                       \
    do {                                                       \
        ++checks;                                              \
        if (!(cond)) {                                         \
            ++failures;                                        \
            std::printf("FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                          \
            std::printf("\n");                                 \
        }                                                      \
    } while (0)

struct Sample {
    double t, h, v, a, pressure, axial;
};

/// 1-D vertical flight: 3 s boost at 75 m/s^2, quadratic drag, drogue after apogee (25 m/s),
/// main below 600 m (6 m/s). Sensors at 100 Hz with noise.
std::vector<Sample> syntheticFlight(unsigned seed, double& trueApogeeTime, double& trueApogee) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0.0, 1.0);
    const double g = 9.80665, k = 0.00025, p0 = 101325.0;
    double h = 0, v = 0, t = 0;
    bool descending = false;
    trueApogee = 0;
    trueApogeeTime = -1;
    std::vector<Sample> out;
    const double dt = 0.001;
    double nextSample = 0.0, landedAt = -1.0;
    while (t < 400.0) {
        const bool burning = t >= 3.0 && t < 6.0;  // 3 s on the pad first
        double a;
        if (!descending) {
            a = (burning ? 75.0 : 0.0) - g - k * v * std::fabs(v) * (h > 0 || burning ? 1.0 : 0.0);
            if (h <= 0 && !burning && t < 3.0) a = 0.0;
        } else {
            const double vt = h > 600.0 ? -25.0 : -6.0;
            a = (vt - v) / 1.5;  // relax towards the terminal velocity
        }
        const double vOld = v;
        v += a * dt;
        h += v * dt;
        t += dt;
        if (h < 0) {
            h = 0;
            v = std::max(v, 0.0);
            if (descending && landedAt < 0) landedAt = t;
        }
        if (landedAt > 0) {
            v = 0;
            a = 0;
            if (t > landedAt + 10.0) break;  // keep sampling on the ground for 10 s
        }
        if (!descending && vOld > 0 && v <= 0 && t > 4.0) {
            descending = true;
            trueApogee = h;
            trueApogeeTime = t;
        }
        if (t >= nextSample) {
            nextSample += 0.01;
            const double p = p0 * std::pow(1.0 - h / 44330.77, 1.0 / 0.190263) + 3.0 * n(rng);
            // Accelerometer along the axis: specific force (a + g) while the rocket points up;
            // under parachute the sensor tumbles, give it noise only.
            const double axial = descending ? g + (landedAt > 0 ? 0.3 : 2.0) * n(rng) : a + g + 0.3 * n(rng);
            out.push_back({t, h, v, a, p, axial});
        }
    }
    return out;
}

void testFlight(unsigned seed) {
    double tApo, hApo;
    const std::vector<Sample> s = syntheticFlight(seed, tApo, hApo);
    rufilters::FlightComputer<float> fc;
    double launch = -1, burnout = -1, apogee = -1, mainT = -1, landed = -1, mainTrueAlt = -1;
    double sumErr2 = 0;
    int nErr = 0;
    for (const Sample& x : s) {
        const uint8_t ev = fc.update(static_cast<float>(x.t), static_cast<float>(x.pressure), static_cast<float>(x.axial));
        if (ev & rufilters::EventLaunch) launch = x.t;
        if (ev & rufilters::EventBurnout) burnout = x.t;
        if (ev & rufilters::EventApogee) apogee = x.t;
        if (ev & rufilters::EventMain) {
            mainT = x.t;
            mainTrueAlt = x.h;
        }
        if (ev & rufilters::EventLanded) landed = x.t;
        if (x.t > 7.0 && x.t < tApo) {
            sumErr2 += (fc.velocity() - x.v) * (fc.velocity() - x.v);
            ++nErr;
        }
    }
    const double rms = std::sqrt(sumErr2 / nErr);
    CHECK(launch > 3.0 && launch < 3.15, "seed %u launch at %.3f", seed, launch);
    CHECK(burnout > 6.0 && burnout < 6.2, "seed %u burnout at %.3f", seed, burnout);
    CHECK(apogee > tApo && apogee < tApo + 1.5, "seed %u apogee at %.2f (true %.2f)", seed, apogee, tApo);
    CHECK(std::fabs(fc.detector().apogeeAltitude() - hApo) < 3.0, "seed %u apogee altitude %.1f (true %.1f)", seed,
          fc.detector().apogeeAltitude(), hApo);
    CHECK(mainT > 0 && std::fabs(mainTrueAlt - 600.0) < 25.0, "seed %u main at true altitude %.1f", seed, mainTrueAlt);
    CHECK(landed > 0, "seed %u landing not detected", seed);
    CHECK(rms < 2.0, "seed %u velocity RMS error %.2f m/s", seed, rms);
}

void testKalmanConvergence() {
    // Constant velocity climb seen only through a noisy barometer.
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0.0, 1.0);
    rufilters::AltitudeKalman<double> kf(1.0, 1.0, 0.5);
    for (int i = 0; i < 2000; ++i) {
        const double t = i * 0.01;
        kf.predict(0.01);
        kf.updateAltitude(20.0 * t + n(rng));
    }
    CHECK(std::fabs(kf.velocity() - 20.0) < 0.5, "velocity %.3f", kf.velocity());
    CHECK(std::fabs(kf.acceleration()) < 0.5, "acceleration %.3f", kf.acceleration());
    CHECK(kf.sigma(0) < 0.5, "altitude sigma %.3f", kf.sigma(0));
}

void testBasics() {
    CHECK(std::fabs(rufilters::pressureToAltitude<double>(89874.6, 101325.0) - 1000.0) < 1.0, "1000 m");
    rufilters::LowPass<double> lp(0.5);
    double y = 0;
    for (int i = 0; i < 1000; ++i) y = lp.update(i == 0 ? 0.0 : 1.0, 0.01);
    CHECK(std::fabs(y - 1.0) < 1e-6, "low pass %.6f", y);
    rufilters::Calibrator<double> c(4);
    c.add(1);
    c.add(2);
    c.add(3);
    CHECK(c.add(6) && c.mean() == 3.0, "calibrator");
}

}  // namespace

int main() {
    testBasics();
    testKalmanConvergence();
    for (unsigned seed = 1; seed <= 20; ++seed) testFlight(seed);
    std::printf("%d/%d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
