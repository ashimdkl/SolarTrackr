#pragma once
// neo-6m over uart. gives utc time and position. between fixes, time is carried
// forward with millis() for up to `holdoverS`, so a short loss of sky view is fine.
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "core/interfaces.h"

class GpsTime : public ITimeSource {
 public:
  GpsTime();
  void begin(HardwareSerial& port, int rxPin, int txPin, uint32_t baud, uint32_t holdoverS);
  void service();  // call often, parses whatever bytes arrived
  bool utcNow(UtcTime& out) override;
  bool location(double& lat, double& lng) override;

  bool started() const { return port_ != nullptr; }
  bool everSynced() const { return synced_; }
  uint32_t satellites() { return gps_.satellites.isValid() ? gps_.satellites.value() : 0; }
  uint32_t charsProcessed() { return gps_.charsProcessed(); }

 private:
  HardwareSerial* port_;
  TinyGPSPlus gps_;
  bool synced_;
  int64_t syncEpoch_;
  uint32_t syncMillis_;
  uint32_t holdoverMs_;
};
