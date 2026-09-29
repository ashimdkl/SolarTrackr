#pragma once
// one actuator's h-bridge. supports the three common wiring styles because the
// driver part is still unconfirmed (see config.h MOTOR_MODE).
#include <Arduino.h>
#include "config.h"
#include "core/interfaces.h"

class MotorDriver : public IAxisActuator {
 public:
  MotorDriver(cfg::MotorDriverMode mode, int pinA, int pinB, int pinC, bool invert);
  bool begin();  // false if mode or required pins are not set
  bool configured() const { return ok_; }
  const char* problem() const { return problem_; }

  void drive(int dir, double duty) override;  // +1 extend, -1 retract
  void stop() override;

 private:
  bool attachPwm(int pin, int& channel);
  void writePwm(int pin, int channel, double duty);

  cfg::MotorDriverMode mode_;
  int pinA_, pinB_, pinC_;
  int chA_, chB_, chC_;
  bool invert_;
  bool ok_;
  const char* problem_;
};
