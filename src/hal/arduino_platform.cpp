#include "arduino_platform.h"
#include <stdarg.h>

void ArduinoClock::sleepMs(uint32_t ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) {
    if (hook_) hook_();
    delay(1);
  }
}

void SerialLogger::logf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}
