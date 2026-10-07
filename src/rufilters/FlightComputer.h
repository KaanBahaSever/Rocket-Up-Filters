#pragma once

// Flight phase detection (launch, burnout, apogee, main altitude, landing) and a ready-made
// flight computer that combines ground calibration, the Kalman filter and the detectors.

#include <math.h>
#include <stdint.h>

#include "AltitudeKalman.h"
#include "Basics.h"

namespace rufilters {

enum class Phase : uint8_t { Calibrating, Pad, Boost, Coast, Drogue, Main, Landed };

/// Bit flags returned by `update()` for the events detected in that call.
enum Event : uint8_t { EventNone = 0, EventLaunch = 1, EventBurnout = 2, EventApogee = 4, EventMain = 8, EventLanded = 16 };

inline const char* phaseName(Phase p) {
    switch (p) {
        case Phase::Calibrating: return "calibrating";
        case Phase::Pad: return "pad";
        case Phase::Boost: return "boost";
        case Phase::Coast: return "coast";
        case Phase::Drogue: return "drogue";
        case Phase::Main: return "main";
        case Phase::Landed: return "landed";
    }
    return "?";
}

template <typename T = float>
struct DetectorConfig {
    T launchAccel = T(3.0 * 9.81);  ///< axial specific force threshold (m/s^2)
    int launchSamples = 5;          ///< consecutive samples above the threshold
    T launchAltitude = T(30);       ///< backup launch detection on altitude (m)
    T apogeeLockout = T(5);         ///< s after launch before apogee may be declared
    T minApogeeAltitude = T(30);    ///< m; ignore "apogees" below this
    int apogeeSamples = 5;          ///< consecutive samples with negative velocity
    T apogeeDrop = T(3);            ///< m below the maximum altitude (second condition)
    T mainAltitude = T(600);        ///< m above the pad
    T landedSpeed = T(2);           ///< m/s
    T landedTime = T(5);            ///< s below landedSpeed near the ground
    T landedAltitude = T(30);       ///< m
};

/// State machine working on filtered altitude/velocity and raw axial acceleration.
template <typename T = float>
class FlightDetector {
public:
    explicit FlightDetector(const DetectorConfig<T>& c = DetectorConfig<T>()) : c_(c) {}

    uint8_t update(T time, T axialAccel, T altitude, T velocity) {
        uint8_t ev = EventNone;
        if (altitude > maxAltitude_) maxAltitude_ = altitude;
        switch (phase_) {
            case Phase::Calibrating:
            case Phase::Pad:
                count_ = axialAccel > c_.launchAccel ? count_ + 1 : 0;
                if (count_ >= c_.launchSamples || altitude > c_.launchAltitude) {
                    phase_ = Phase::Boost;
                    launchTime_ = time;
                    count_ = 0;
                    ev |= EventLaunch;
                }
                break;
            case Phase::Boost:
                if (axialAccel < T(0)) {  // drag only: the motor is out
                    phase_ = Phase::Coast;
                    burnoutTime_ = time;
                    ev |= EventBurnout;
                }
                // fall through - apogee detection also runs in the boost phase (very short burns)
            case Phase::Coast:
                if (time - launchTime_ > c_.apogeeLockout && maxAltitude_ > c_.minApogeeAltitude) {
                    count_ = velocity < T(0) ? count_ + 1 : 0;
                    if (count_ >= c_.apogeeSamples && maxAltitude_ - altitude > c_.apogeeDrop) {
                        if (phase_ == Phase::Boost) ev |= EventBurnout;
                        phase_ = Phase::Drogue;
                        apogeeTime_ = time;
                        apogeeAltitude_ = maxAltitude_;
                        count_ = 0;
                        ev |= EventApogee;
                    }
                }
                break;
            case Phase::Drogue:
                if (altitude < c_.mainAltitude) {
                    phase_ = Phase::Main;
                    ev |= EventMain;
                }
                break;
            case Phase::Main:
                if (fabs(velocity) < c_.landedSpeed && altitude < c_.landedAltitude) {
                    if (stillSince_ < T(0)) stillSince_ = time;
                    if (time - stillSince_ > c_.landedTime) {
                        phase_ = Phase::Landed;
                        ev |= EventLanded;
                    }
                } else {
                    stillSince_ = T(-1);
                }
                break;
            case Phase::Landed: break;
        }
        return ev;
    }

    Phase phase() const { return phase_; }
    void setPadReady() {
        if (phase_ == Phase::Calibrating) phase_ = Phase::Pad;
    }
    T launchTime() const { return launchTime_; }
    T burnoutTime() const { return burnoutTime_; }
    T apogeeTime() const { return apogeeTime_; }
    T apogeeAltitude() const { return apogeeAltitude_; }
    T maxAltitude() const { return maxAltitude_; }
    const DetectorConfig<T>& config() const { return c_; }

private:
    DetectorConfig<T> c_;
    Phase phase_ = Phase::Calibrating;
    int count_ = 0;
    T launchTime_ = T(-1), burnoutTime_ = T(-1), apogeeTime_ = T(-1), apogeeAltitude_ = T(0);
    T maxAltitude_ = T(-1e9);
    T stillSince_ = T(-1);
};

/// Barometer + accelerometer flight computer for a rocket flying roughly vertically.
///
///   rufilters::FlightComputer<float> fc;
///   uint8_t ev = fc.update(t, pressurePa, axialAccel);   // every sample
///   if (ev & rufilters::EventApogee) fireDrogue();
///   if (ev & rufilters::EventMain)   fireMain();
///
/// The accelerometer is used during powered and coasting flight (vertical acceleration is
/// taken as axialAccel - g); under parachute only the barometer is used.
template <typename T = float>
class FlightComputer {
public:
    struct Config {
        DetectorConfig<T> detector;
        int calibrationSamples = 50;  ///< ground pressure averaging
        T jerkNoise = T(50);
        T altitudeNoise = T(0.5);
        T accelNoise = T(1.0);
        T gravity = T(9.80665);
    };

    FlightComputer() : FlightComputer(Config()) {}
    explicit FlightComputer(const Config& c)
        : c_(c), cal_(c.calibrationSamples), kf_(c.jerkNoise, c.altitudeNoise, c.accelNoise), det_(c.detector) {}

    /// `pressure` in Pa, `axialAccel` = accelerometer reading along the rocket axis (m/s^2,
    /// +g when standing on the pad). Returns the Event bits detected in this call.
    uint8_t update(T time, T pressure, T axialAccel) {
        if (!cal_.done()) {
            if (cal_.add(pressure)) {
                groundPressure_ = cal_.mean();
                kf_.reset(T(0));
                det_.setPadReady();
            }
            lastTime_ = time;
            return EventNone;
        }
        const T dt = time - lastTime_;
        lastTime_ = time;
        kf_.predict(dt > T(0) && dt < T(1) ? dt : T(0.01));
        kf_.updateAltitude(pressureToAltitude<T>(pressure, groundPressure_));
        const Phase p = det_.phase();
        if (p == Phase::Pad || p == Phase::Boost || p == Phase::Coast) kf_.updateAcceleration(axialAccel - c_.gravity);
        return det_.update(time, axialAccel, kf_.altitude(), kf_.velocity());
    }

    Phase phase() const { return det_.phase(); }
    T altitude() const { return kf_.altitude(); }
    T velocity() const { return kf_.velocity(); }
    T acceleration() const { return kf_.acceleration(); }
    T groundPressure() const { return groundPressure_; }
    const AltitudeKalman<T>& filter() const { return kf_; }
    const FlightDetector<T>& detector() const { return det_; }

private:
    Config c_;
    Calibrator<T> cal_;
    AltitudeKalman<T> kf_;
    FlightDetector<T> det_;
    T groundPressure_ = T(101325);
    T lastTime_ = T(0);
};

}  // namespace rufilters
