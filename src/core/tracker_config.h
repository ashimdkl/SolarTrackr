#pragma once
// runtime copy of include/config.h. the tracker only reads this struct, so the
// simulator can override values (e.g. assumed actuator lengths) without touching config.h.
#include <stdint.h>
#include "types.h"
#include "axis_geometry.h"
#include "energy_model.h"

struct EncoderCal {
  int zeroRaw;  // raw count with panel flat
  int dir;      // +1 / -1
};

struct TrackerConfig {
  double fallbackLat, fallbackLng;
  uint32_t cyclePeriodS, pollMs;

  double minTrackElevDeg, stowTiltDeg, stowAzDeg;
  double deadbandDeg, slowZoneDeg, dutyFull, dutySlow;
  int maxReversals;

  double stallWindowS, stallMinProgressDeg, timeoutFactor, timeoutExtraS, overcurrentA;
  int sensorRetries;
  double sensorRangeSlackDeg;
  int faultCooldownCycles, maxFaultsBeforeLockout;

  EnergyConfig energy;
  double movePowerW;
  bool allowMotionWithoutEnergyData;

  double windStowMps;
  uint32_t windReleaseS;

  AxisGeometryConfig axis[AXIS_COUNT];
  EncoderCal enc[AXIS_COUNT];
};

// builds the struct from include/config.h
TrackerConfig configFromHeader();

// collects everything that is missing or wrong. blocking items stop all motion.
struct ConfigCheck {
  static const int kMaxItems = 24;
  bool motionAllowed;
  int count;
  char items[kMaxItems][112];

  ConfigCheck() : motionAllowed(true), count(0) {}
  void add(bool blocksMotion, const char* fmt, ...);
};

void validateConfig(const TrackerConfig& c, ConfigCheck& out);
