#include "encoder_as5600.h"
#include <math.h>

// registers
static const uint8_t REG_STATUS = 0x0B;     // bit5 MD magnet detected, bit4 ML too weak, bit3 MH too strong
static const uint8_t REG_AGC = 0x1A;        // agc, then MAGNITUDE 0x1B..0x1C
// burst read from STATUS covers RAW_ANGLE (0x0C..0x0D) and ANGLE (0x0E..0x0F)

As5600Sensor::As5600Sensor(TwoWire& wire, uint8_t addr, int zeroRaw, int dir)
    : wire_(wire), addr_(addr), zeroRaw_(zeroRaw), dir_(dir), enabled_(false) {}

bool As5600Sensor::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom(addr_, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)wire_.read();
  return true;
}

bool As5600Sensor::readRaw(uint16_t& angleRaw, uint8_t& status, uint8_t& agc, uint16_t& magnitude) {
  if (!enabled_) return false;
  uint8_t a[5];
  uint8_t b[3];
  if (!readRegs(REG_STATUS, a, 5)) return false;
  if (!readRegs(REG_AGC, b, 3)) return false;
  status = a[0];
  angleRaw = (uint16_t)(((a[3] & 0x0F) << 8) | a[4]);  // ANGLE register
  agc = b[0];
  magnitude = (uint16_t)(((b[1] & 0x0F) << 8) | b[2]);
  return true;
}

SensorReading As5600Sensor::read() {
  SensorReading r;
  r.commOk = false;
  r.magnetOk = false;
  r.raw = 0;
  r.angleDeg = 0;
  r.agc = 0;
  r.magnitude = 0;

  uint8_t status = 0;
  if (!readRaw(r.raw, status, r.agc, r.magnitude)) return r;
  r.commOk = true;
  bool md = status & 0x20, ml = status & 0x10, mh = status & 0x08;
  r.magnetOk = md && !ml && !mh;

  double deg = r.raw * 360.0 / 4096.0;
  if (calibrated()) {
    double rel = (int(r.raw) - zeroRaw_) * 360.0 / 4096.0;
    rel = fmod(rel + 540.0, 360.0) - 180.0;  // wrap to [-180, 180)
    r.angleDeg = rel * dir_;
  } else {
    r.angleDeg = deg;  // uncalibrated, motion is blocked anyway
  }
  return r;
}
