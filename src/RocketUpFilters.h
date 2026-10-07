#pragma once

// Rocket-Up-Filters: state estimation and flight-event detection for rocket flight computers.
// Header-only, C++11, no dynamic memory and no STL, so it builds for Arduino (AVR, ESP32,
// Teensy, RP2040, STM32) as well as for desktop tests.

#include "rufilters/AltitudeKalman.h"
#include "rufilters/Basics.h"
#include "rufilters/FlightComputer.h"

#define RUFILTERS_VERSION "0.2.0"
