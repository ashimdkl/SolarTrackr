#pragma once
// pulse output anemometer (reed switch, one or more pulses per revolution).
// poll() computes speed over 3 s windows and keeps the peak gust; speedMps() returns
// the peak since the last call. calibration factor comes from the anemometer datasheet.
#include <Arduino.h>
#include "core/interfaces.h"

class PulseAnemometer : public IWindSensor {
 public:
  PulseAnemometer() : pin_(-1), k_(-1), windowStartMs_(0), windowStartPulses_(0), gust_(-1) {}
  bool begin(int pin, double hzToMps);
  bool active() const { return pin_ >= 0 && k_ > 0; }
  void poll();
  double peekGust() const { return gust_; }
  bool speedMps(double& out) override;

 private:
  static void IRAM_ATTR onPulse();
  static volatile uint32_t pulses_;
  static volatile uint32_t lastPulseUs_;

  int pin_;
  double k_;
  uint32_t windowStartMs_;
  uint32_t windowStartPulses_;
  double gust_;
};
