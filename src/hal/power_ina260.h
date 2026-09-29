#pragma once
// ina260 on the actuator supply. measures what a move actually costs.
#include <Adafruit_INA260.h>
#include <Wire.h>
#include "core/interfaces.h"

class Ina260Monitor : public IPowerMonitor {
 public:
  Ina260Monitor() : ok_(false) {}
  bool begin(TwoWire* wire, uint8_t addr) {
    ok_ = ina_.begin(addr, wire);
    return ok_;
  }
  bool ok() const { return ok_; }
  PowerReading read() override {
    PowerReading p;
    p.ok = ok_;
    p.volts = p.amps = p.watts = 0;
    if (!ok_) return p;
    p.volts = ina_.readBusVoltage() / 1000.0;
    p.amps = ina_.readCurrent() / 1000.0;  // sign depends on wiring direction, callers use fabs
    p.watts = ina_.readPower() / 1000.0;
    return p;
  }

 private:
  Adafruit_INA260 ina_;
  bool ok_;
};
