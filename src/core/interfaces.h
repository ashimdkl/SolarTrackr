#pragma once
// hardware seams. the esp32 build implements these with real drivers (src/hal),
// the native sim implements them with models (src/sim). core logic only sees these.
#include "types.h"

class IAxisSensor {
 public:
  virtual ~IAxisSensor() {}
  virtual SensorReading read() = 0;
};

class IAxisActuator {
 public:
  virtual ~IAxisActuator() {}
  // dir: +1 extend, -1 retract. duty 0..1
  virtual void drive(int dir, double duty) = 0;
  virtual void stop() = 0;
};

class IClock {
 public:
  virtual ~IClock() {}
  virtual uint32_t nowMs() = 0;   // wraps after ~49 days, always compare with subtraction
  virtual void sleepMs(uint32_t ms) = 0;
};

class ITimeSource {
 public:
  virtual ~ITimeSource() {}
  virtual bool utcNow(UtcTime& out) = 0;
  virtual bool location(double& lat, double& lng) = 0;
};

class IPowerMonitor {
 public:
  virtual ~IPowerMonitor() {}
  virtual PowerReading read() = 0;
};

class IWindSensor {
 public:
  virtual ~IWindSensor() {}
  virtual bool speedMps(double& out) = 0;
};

class ILogger {
 public:
  virtual ~ILogger() {}
  virtual void logf(const char* fmt, ...) = 0;
};
