#include "wind_sensor.h"

volatile uint32_t PulseAnemometer::pulses_ = 0;
volatile uint32_t PulseAnemometer::lastPulseUs_ = 0;

void IRAM_ATTR PulseAnemometer::onPulse() {
  uint32_t now = micros();
  if (now - lastPulseUs_ > 5000) {  // 5 ms reed switch debounce
    pulses_++;
    lastPulseUs_ = now;
  }
}

bool PulseAnemometer::begin(int pin, double hzToMps) {
  if (pin < 0 || hzToMps <= 0) return false;
  pin_ = pin;
  k_ = hzToMps;
  pinMode(pin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_), onPulse, FALLING);
  windowStartMs_ = millis();
  windowStartPulses_ = pulses_;
  return true;
}

void PulseAnemometer::poll() {
  if (!active()) return;
  uint32_t now = millis();
  uint32_t dt = now - windowStartMs_;
  if (dt < 3000) return;
  uint32_t p = pulses_;
  double hz = (p - windowStartPulses_) * 1000.0 / dt;
  double mps = hz * k_;
  if (mps > gust_) gust_ = mps;
  windowStartMs_ = now;
  windowStartPulses_ = p;
}

bool PulseAnemometer::speedMps(double& out) {
  if (!active() || gust_ < 0) return false;
  out = gust_;
  gust_ = -1;
  return true;
}
