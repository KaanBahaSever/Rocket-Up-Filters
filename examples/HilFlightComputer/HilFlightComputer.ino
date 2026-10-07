/*
  Rocket-Up-Filters: hardware-in-the-loop flight computer.

  Runs the Kalman-filter flight computer of this library on an Arduino and lets the Rocket-Up
  simulator feed it with simulated sensors over USB serial:

    rocketup hil data/projects/orbit_chaser_hil.project.json --port COM3

  The board answers $RUCMD,drogue at apogee and $RUCMD,main below 600 m, which deploy the
  parachutes in the simulation. Replace handleSensors() input with your real barometer and
  accelerometer readings to fly the same code.

  Protocol: https://github.com/KaanBahaSever/Rocket-Up/blob/main/docs/HIL.md
*/

#include <RocketUpFilters.h>

const long BAUD = 115200;
const int PIN_DROGUE = 5;
const int PIN_MAIN = 6;
#ifdef LED_BUILTIN
const int PIN_LED = LED_BUILTIN;
#else
const int PIN_LED = 2;  // boards without a builtin LED definition (e.g. generic ESP32)
#endif

rufilters::FlightComputer<float> computer;
char line[200];
int lineLen = 0;

byte checksum(const char* s) {
  byte c = 0;
  while (*s) c ^= (byte)*s++;
  return c;
}

void sendFrame(const char* body) {
  char buf[64];
  snprintf(buf, sizeof(buf), "$%s*%02X", body, checksum(body));
  Serial.println(buf);
}

void logValue(const char* text, float value) {
  Serial.print("$RULOG,");
  Serial.print(text);
  Serial.print(' ');
  Serial.println(value, 1);
}

void handleSensors(char* body) {
  // RUSEN,t_ms,p_Pa,T_C,ax,ay,az,gx,gy,gz,baroAlt,fix,lat,lon,gpsAlt
  float f[15];
  int n = 0;
  char* tok = strtok(body, ",");
  while ((tok = strtok(NULL, ",")) != NULL && n < 15) f[n++] = atof(tok);
  if (n < 4) return;
  const float t = f[0] / 1000.0f;
  const float pressure = f[1];
  const float axial = f[3];

  const uint8_t ev = computer.update(t, pressure, axial);
  if (ev & rufilters::EventLaunch) logValue("launch at t =", t);
  if (ev & rufilters::EventBurnout) logValue("burnout, velocity", computer.velocity());
  if (ev & rufilters::EventApogee) {
    digitalWrite(PIN_DROGUE, HIGH);
    sendFrame("RUCMD,drogue");
    logValue("apogee", computer.detector().apogeeAltitude());
  }
  if (ev & rufilters::EventMain) {
    digitalWrite(PIN_MAIN, HIGH);
    sendFrame("RUCMD,main");
    logValue("main at", computer.altitude());
  }
  if (ev & rufilters::EventLanded) logValue("landed at t =", t);
}

void handleLine(char* s) {
  if (s[0] != '$') return;
  char* star = strchr(s, '*');
  if (star) {
    *star = '\0';
    if (checksum(s + 1) != (byte)strtol(star + 1, NULL, 16)) return;
  }
  char* body = s + 1;
  if (strncmp(body, "RUSEN,", 6) == 0) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    handleSensors(body);
  } else if (strncmp(body, "RUHELLO", 7) == 0) {
    computer = rufilters::FlightComputer<float>();
    digitalWrite(PIN_DROGUE, LOW);
    digitalWrite(PIN_MAIN, LOW);
    Serial.println("$RULOG,Rocket-Up-Filters flight computer ready");
  }
}

void setup() {
  pinMode(PIN_DROGUE, OUTPUT);
  pinMode(PIN_MAIN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(BAUD);
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        line[lineLen] = '\0';
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}
