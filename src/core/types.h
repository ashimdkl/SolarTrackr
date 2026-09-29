#pragma once
// shared plain data types. no arduino, compiles on esp32 and native.
#include <stdint.h>

enum AxisId { AXIS_TILT = 0, AXIS_AZ = 1, AXIS_COUNT = 2 };

inline const char* axisName(int axis) { return axis == AXIS_TILT ? "tilt" : "azimuth"; }

struct UtcTime {
  int year, month, day;
  int hour, minute, second;
};

// one encoder read
struct SensorReading {
  bool commOk;         // i2c transaction worked (false = wiring / cable fault)
  bool magnetOk;       // magnet detected and field in range (false = magnet shifted / failing)
  uint16_t raw;        // 0..4095
  double angleDeg;     // calibrated joint angle, 0 = panel flat
  uint8_t agc;         // as5600 automatic gain, drift here is an early warning
  uint16_t magnitude;  // as5600 field magnitude
};

struct PowerReading {
  bool ok;
  double volts, amps, watts;
};
