#pragma once
// as5600 magnetic angle sensor. fixed i2c address 0x36, so each one gets its own bus
// (Wire for tilt, Wire1 for azimuth).
#include <Arduino.h>
#include <Wire.h>
#include "core/interfaces.h"

class As5600Sensor : public IAxisSensor {
 public:
  As5600Sensor(TwoWire& wire, uint8_t addr, int zeroRaw, int dir);
  void setEnabled(bool on) { enabled_ = on; }  // false until its i2c bus is started
  bool enabled() const { return enabled_; }
  bool calibrated() const { return zeroRaw_ >= 0 && zeroRaw_ <= 4095 && (dir_ == 1 || dir_ == -1); }

  SensorReading read() override;

  // raw register dump for calibration (console "raw")
  bool readRaw(uint16_t& angleRaw, uint8_t& status, uint8_t& agc, uint16_t& magnitude);

 private:
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);

  TwoWire& wire_;
  uint8_t addr_;
  int zeroRaw_;
  int dir_;
  bool enabled_;
};
