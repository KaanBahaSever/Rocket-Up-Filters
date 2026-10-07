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

Sensors usually run at different rates: an MPU6050 can be read at 200 Hz, a BMP280 delivers new pressure at ~50�70 Hz.
Call `update()` at the IMU rate and pass `pressureIsNew = false` when the barometer has no new sample:

```cpp
uint8_t ev = computer.update(t, lastPressure, axial, /*pressureIsNew=*/baroWasRead);
```

### BMP280 + MPU6050

[`examples/BMP280_MPU6050`](examples/BMP280_MPU6050/BMP280_MPU6050.ino) is a complete flight computer for this sensor pair
(Adafruit libraries). It runs the IMU at 200 Hz and the barometer at 50 Hz, and checks the sensor orientation at startup.
It drives pulsed pyro outputs with an optional arming switch and streams a CSV log over serial. Before you fly:

1. **Orientation.** The startup message must report an axial acceleration of about +9.8 m/s� on the pad. Otherwise, change
   `AXIS` and `AXIS_SIGN`.
2. **Calibration.** Keep the rocket still for ~2 s after power-up while the ground pressure is calibrated.
3. **Deployment logic.** Test it with LEDs on the pyro pins, and against the simulator, before connecting igniters.
4. **Barometer mounting.** Put the BMP280 in a bay with static vent holes, away from ejection-charge gases.

## Components

| Class | Purpose |
|---|---|
| `AltitudeKalman<T>` | Constant-acceleration Kalman filter with `predict(dt)`, `updateAltitude(h)` and `updateAcceleration(a)`. It uses scalar updates only (no matrix inversion), exposes per-state sigmas and the normalised innovation for glitch detection |
| `FlightDetector<T>` | State machine `Pad → Boost → Coast → Drogue → Main → Landed` driven by filtered altitude and velocity plus raw acceleration |
| `FlightComputer<T>` | Ground calibration, filter and detector in one `update(t, pressure, axialAccel[, pressureIsNew])` call |
| `pressureToAltitude`, `LowPass`, `Calibrator` | Barometric altitude, first-order filter, ground averaging |

Tuning lives in `FlightComputer<T>::Config` and `DetectorConfig<T>`: launch threshold, apogee lock-out time, confirmation
times (independent of the sample rate), main altitude, and the noise levels of the filter.

## Test against the simulator

Hardware in the loop: flash `examples/HilFlightComputer`, then in Rocket-Up run

```bash
rocketup hil data/projects/orbit_chaser_hil.project.json --port COM3
```

The simulator sends barometer, IMU and GPS data. The board answers `$RUCMD,drogue` and `$RUCMD,main`, which open the parachutes
in the simulation, and the flight report shows what happened.

### Replaying real logs on a PC

`replay_log` runs the flight computer on any recorded sensor log: real flights, ground tests, or logs of other flight
computers. Use it to tune the thresholds and noise settings on real data:

```bash
replay_log flight.csv                                   # log of examples/BMP280_MPU6050
replay_log other.csv --time time_s --time-scale 1 --pressure press_hPa --pressure-scale 100                      --accel acc_z_g --accel-scale 9.80665 --main 500
```

- Columns are selected by header name or index, and the delimiter is detected automatically.
- The output lists the detected events, and `<log>_filtered.csv` holds the raw barometric altitude, the filtered altitude,
  velocity, acceleration and phase for plotting.

`replay_rocketup` does the same for a Rocket-Up flight CSV: it adds sensor noise and compares the detected events with the true ones.

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

The tests fly 30 synthetic noisy flights. Ten of them run the IMU at 200 Hz and the barometer at 50 Hz. For each one they check launch and burnout timing, that apogee is detected after the
true apogee and within 1.5 s and 3 m of it, that the main fires at 600 ± 25 m, landing detection, and the velocity error
(RMS < 2 m/s). The tests compile as C++11 so the headers stay usable with embedded toolchains.

## Roadmap

- Attitude estimation (gyro integration with accelerometer and magnetometer correction) for tilt-compensated vertical acceleration
- Temperature-compensated barometric altitude
- GPS fusion for horizontal position and landing prediction

## License

MIT. See [LICENSE](LICENSE).
