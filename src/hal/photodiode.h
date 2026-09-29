#pragma once
// photodiode + transimpedance amp, powered through the tps22917 load switch.
// validation only: logged each cycle to compare against sun_math on clear days.
#include <Arduino.h>

class Photodiode {
 public:
  Photodiode() : adcPin_(-1), enPin_(-1) {}
  bool begin(int adcPin, int enablePin) {
    adcPin_ = adcPin;
    enPin_ = enablePin;
    if (adcPin_ < 0) return false;
    if (enPin_ >= 0) {
      pinMode(enPin_, OUTPUT);
      digitalWrite(enPin_, LOW);
    }
    return true;
  }
  bool active() const { return adcPin_ >= 0; }
  bool readMilliVolts(uint32_t& mv) {
    if (adcPin_ < 0) return false;
    if (enPin_ >= 0) {
      digitalWrite(enPin_, HIGH);
      delay(20);  // let the amp settle
    }
    mv = analogReadMilliVolts(adcPin_);
    if (enPin_ >= 0) digitalWrite(enPin_, LOW);
    return true;
  }

 private:
  int adcPin_, enPin_;
};
