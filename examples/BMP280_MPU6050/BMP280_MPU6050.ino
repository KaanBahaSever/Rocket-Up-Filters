/*
  Rocket-Up-Filters flight computer with real sensors: BMP280 barometer + MPU6050 IMU.

  - MPU6050 read at 200 Hz (+-16 g, +-500 deg/s), BMP280 at 50 Hz, both over I2C (400 kHz).
  - Kalman filter fuses both; detects launch, burnout, apogee, main altitude and landing.
  - Fires the drogue at apogee and the main below MAIN_ALTITUDE (pulsed pyro outputs).
  - Streams a CSV log over Serial that tools/replay_log in this repository can re-run on a PC.

  Wiring (Uno/Nano): SDA -> A4, SCL -> A5, VCC 3.3 V, GND. ESP32: SDA 21, SCL 22.
  BMP280 address 0x76 (SDO to GND) or 0x77 (SDO to VCC).

  Libraries (Library Manager): "Adafruit BMP280 Library", "Adafruit MPU6050".

  BEFORE FLIGHT
   1. Mount the board and check the startup message: the axial acceleration must read about
      +9.8 m/s^2 with the rocket standing on the pad. If it reads -9.8, flip AXIS_SIGN; if it
      reads ~0, change AXIS (the MPU6050 axis that points to the nose).
   2. Keep the rocket still for ~2 s after power-up: the ground pressure is calibrated then.
   3. Test the deployment logic with LEDs on the pyro pins, and against the simulator
      (examples/HilFlightComputer + `rocketup hil ...`), before connecting igniters.
*/

#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <RocketUpFilters.h>
#include <Wire.h>

// ------------------------------------------------------------------ configuration
const int AXIS = 2;            // MPU6050 axis pointing to the nose: 0 = X, 1 = Y, 2 = Z
const float AXIS_SIGN = 1.0f;  // -1 if the sensor is mounted upside down
const uint8_t BMP_ADDRESS = 0x76;
const float MAIN_ALTITUDE = 600.0f;  // m above the pad

const int PIN_DROGUE = 5;
const int PIN_MAIN = 6;
const int PIN_ARM = -1;             // arming switch to GND (INPUT_PULLUP); -1 = always armed
const unsigned long PYRO_PULSE_MS = 1000;
const bool LOG_TO_SERIAL = true;    // CSV log at the barometer rate
const long BAUD = 500000;           // fast enough not to stall the 200 Hz loop while logging

const unsigned long IMU_PERIOD_US = 5000;    // 200 Hz
const unsigned long BARO_PERIOD_US = 20000;  // 50 Hz (the BMP280 settings below give ~70 Hz)

// ------------------------------------------------------------------ state
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
rufilters::FlightComputer<float>* computer = nullptr;

float lastPressure = 0.0f, lastTemperature = 0.0f;
unsigned long nextImu = 0, nextBaro = 0;
unsigned long drogueOffAt = 0, mainOffAt = 0;

bool armed() { return PIN_ARM < 0 || digitalRead(PIN_ARM) == LOW; }

void fire(int pin, unsigned long& offAt) {
  if (!armed()) {
    Serial.println(F("# pyro event while DISARMED - not fired"));
    return;
  }
  digitalWrite(pin, HIGH);
  offAt = millis() + PYRO_PULSE_MS;
}

void halt(const __FlashStringHelper* msg) {
  Serial.println(msg);
  while (true) delay(1000);
}

void setup() {
  pinMode(PIN_DROGUE, OUTPUT);
  pinMode(PIN_MAIN, OUTPUT);
  digitalWrite(PIN_DROGUE, LOW);
  digitalWrite(PIN_MAIN, LOW);
  if (PIN_ARM >= 0) pinMode(PIN_ARM, INPUT_PULLUP);
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  if (!bmp.begin(BMP_ADDRESS)) halt(F("# BMP280 not found (check wiring / address)"));
  // Fast normal mode: pressure x4, temperature x1, light IIR filter (the Kalman filter does
  // the rest; heavy IIR filtering would delay apogee detection).
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_X2, Adafruit_BMP280::STANDBY_MS_1);
  if (!mpu.begin()) halt(F("# MPU6050 not found (check wiring)"));
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  static rufilters::FlightComputer<float>::Config config;
  config.detector.mainAltitude = MAIN_ALTITUDE;
  config.altitudeNoise = 0.4f;  // BMP280, x4 oversampling
  config.accelNoise = 0.6f;     // MPU6050 at +-16 g
  config.calibrationSamples = 100;
  static rufilters::FlightComputer<float> fc(config);
  computer = &fc;

  // Startup orientation check.
  float sum = 0;
  for (int i = 0; i < 50; ++i) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    const float v[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    sum += AXIS_SIGN * v[AXIS];
    delay(5);
  }
  Serial.print(F("# axial acceleration at rest: "));
  Serial.print(sum / 50.0f, 2);
  Serial.println(sum / 50.0f > 8.0f ? F(" m/s^2 (OK)") : F(" m/s^2 (CHECK AXIS / AXIS_SIGN!)"));
  if (LOG_TO_SERIAL) Serial.println(F("t_ms,pressure_Pa,temperature_C,axial,ax,ay,az,gx,gy,gz,altitude,velocity,phase"));
  nextImu = nextBaro = micros();
}

void loop() {
  const unsigned long now = micros();
  if (drogueOffAt && millis() > drogueOffAt) digitalWrite(PIN_DROGUE, LOW), drogueOffAt = 0;
  if (mainOffAt && millis() > mainOffAt) digitalWrite(PIN_MAIN, LOW), mainOffAt = 0;
  if ((long)(now - nextImu) < 0) return;
  nextImu += IMU_PERIOD_US;

  bool baroNew = false;
  if ((long)(now - nextBaro) >= 0) {
    nextBaro += BARO_PERIOD_US;
    lastPressure = bmp.readPressure();  // Pa
    lastTemperature = bmp.readTemperature();
    baroNew = true;
  }
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  const float acc[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  const float axial = AXIS_SIGN * acc[AXIS];
  const float time = now / 1.0e6f;

  const uint8_t ev = computer->update(time, lastPressure, axial, baroNew);
  if (ev & rufilters::EventLaunch) Serial.println(F("# launch"));
  if (ev & rufilters::EventBurnout) Serial.println(F("# burnout"));
  if (ev & rufilters::EventApogee) {
    Serial.print(F("# apogee "));
    Serial.println(computer->detector().apogeeAltitude(), 1);
    fire(PIN_DROGUE, drogueOffAt);
  }
  if (ev & rufilters::EventMain) {
    Serial.println(F("# main"));
    fire(PIN_MAIN, mainOffAt);
  }
  if (ev & rufilters::EventLanded) Serial.println(F("# landed"));

  if (LOG_TO_SERIAL && baroNew) {
    Serial.print(now / 1000UL);
    Serial.print(',');
    Serial.print(lastPressure, 1);
    Serial.print(',');
    Serial.print(lastTemperature, 2);
    Serial.print(',');
    Serial.print(axial, 3);
    for (int i = 0; i < 3; ++i) {
      Serial.print(',');
      Serial.print(acc[i], 3);
    }
    Serial.print(',');
    Serial.print(g.gyro.x, 4);
    Serial.print(',');
    Serial.print(g.gyro.y, 4);
    Serial.print(',');
    Serial.print(g.gyro.z, 4);
    Serial.print(',');
    Serial.print(computer->altitude(), 2);
    Serial.print(',');
    Serial.print(computer->velocity(), 2);
    Serial.print(',');
    Serial.println(rufilters::phaseName(computer->phase()));
  }
}
