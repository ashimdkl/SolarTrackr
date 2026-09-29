#pragma once
// clock and logger for the esp32 build
#include <Arduino.h>
#include "core/interfaces.h"

class ArduinoClock : public IClock {
 public:
  typedef void (*IdleHook)();
  ArduinoClock() : hook_(nullptr) {}
  // called repeatedly while the tracker waits (keeps gps parsing and console abort alive)
  void setIdleHook(IdleHook hook) { hook_ = hook; }
  uint32_t nowMs() override { return millis(); }
  void sleepMs(uint32_t ms) override;

 private:
  IdleHook hook_;
};

class SerialLogger : public ILogger {
 public:
  void logf(const char* fmt, ...) override;
};
