# Rocket-Up-Filters

State estimation and flight-event detection for rocket flight computers. This is the flight-software companion of the
[Rocket-Up](https://github.com/KaanBahaSever/Rocket-Up) simulator.

- **Header-only, C++11, no heap and no STL.** The same code runs on Arduino (AVR, ESP32, Teensy, RP2040, STM32) and on a PC.
- **Kalman filter.** A 3-state filter (altitude, vertical velocity, vertical acceleration) fuses the barometer and the accelerometer.
- **Event detection.** Launch, burnout, apogee, main-deployment altitude and landing, with lock-outs against false triggers.
- **Testable before flight.** Run the code against Rocket-Up through its hardware-in-the-loop serial protocol, or replay any simulated flight on a PC.

## Quick start (Arduino)

Install the library by copying this repository into your `Arduino/libraries` folder. Then:

```cpp
#include <RocketUpFilters.h>

rufilters::FlightComputer<float> computer;

void loop() {
  float t = millis() / 1000.0f;
  float pressure = readBarometerPa();        // your sensor
  float axial = readAccelerometerAxial();    // m/s^2 along the rocket axis, +9.81 on the pad
  uint8_t ev = computer.update(t, pressure, axial);
  if (ev & rufilters::EventApogee) fireDrogue();
  if (ev & rufilters::EventMain)   fireMain();
}
```

The first `calibrationSamples` (50) readings set the ground pressure, so start the loop with the rocket on the pad.
`computer.altitude()`, `velocity()`, `acceleration()` and `phase()` are available at any time.

## Components

| Class | Purpose |
|---|---|
| `AltitudeKalman<T>` | Constant-acceleration Kalman filter with `predict(dt)`, `updateAltitude(h)` and `updateAcceleration(a)`. It uses scalar updates only (no matrix inversion), exposes per-state sigmas and the normalised innovation for glitch detection |
| `FlightDetector<T>` | State machine `Pad → Boost → Coast → Drogue → Main → Landed` driven by filtered altitude and velocity plus raw acceleration |
| `FlightComputer<T>` | Ground calibration, filter and detector in one `update(t, pressure, axialAccel)` call |
| `pressureToAltitude`, `LowPass`, `Calibrator` | Barometric altitude, first-order filter, ground averaging |

Tuning lives in `FlightComputer<T>::Config` and `DetectorConfig<T>`: launch threshold, apogee lock-out time, consecutive-sample
counts, main altitude, and the noise levels of the filter.

## Test against the simulator

Hardware in the loop: flash `examples/HilFlightComputer`, then in Rocket-Up run

```bash
rocketup hil data/projects/orbit_chaser_hil.project.json --port COM3
```

The simulator sends barometer, IMU and GPS data. The board answers `$RUCMD,drogue` and `$RUCMD,main`, which open the parachutes
in the simulation, and the flight report shows what happened.

Replay on a PC: `examples/desktop/replay_rocketup` reads a Rocket-Up flight CSV, adds sensor noise, runs the flight computer and
compares the detected events with the true ones.

```
$ replay_rocketup output/orbit_chaser/flight_orbit_chaser.csv
t =    0.05 s  launch detected
t =    4.10 s  burnout detected
t =   26.26 s  apogee detected: 3033.3 m (true 3130.8 m at 25.44 s)
t =  118.91 s  main deployment altitude, true altitude 617.8 m
```

The 3 % altitude difference in this run is real physics, not a filter error. A barometric altimeter assumes the standard
atmosphere, and that flight was launched from 970 m on a 24 °C day. Event timing is unaffected.

## Building the tests

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

The tests fly 20 synthetic noisy flights. For each one they check launch and burnout timing, that apogee is detected after the
true apogee and within 1.5 s and 3 m of it, that the main fires at 600 ± 25 m, landing detection, and the velocity error
(RMS < 2 m/s). The tests compile as C++11 so the headers stay usable with embedded toolchains.

## Roadmap

- Attitude estimation (gyro integration with accelerometer and magnetometer correction) for tilt-compensated vertical acceleration
- Temperature-compensated barometric altitude
- GPS fusion for horizontal position and landing prediction

## License

MIT. See [LICENSE](LICENSE).
