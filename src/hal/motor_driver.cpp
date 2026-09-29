#include "motor_driver.h"

#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

// arduino-esp32 3.x changed the ledc api (pin based instead of channel based)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define LEDC_PIN_API 1
#else
#define LEDC_PIN_API 0
#endif

static int sNextChannel = 0;

MotorDriver::MotorDriver(cfg::MotorDriverMode mode, int pinA, int pinB, int pinC, bool invert)
    : mode_(mode), pinA_(pinA), pinB_(pinB), pinC_(pinC), chA_(-1), chB_(-1), chC_(-1),
      invert_(invert), ok_(false), problem_("not started") {}

bool MotorDriver::attachPwm(int pin, int& channel) {
#if LEDC_PIN_API
  channel = pin;
  return ledcAttach(pin, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_BITS);
#else
  if (sNextChannel >= 16) return false;
  channel = sNextChannel++;
  ledcSetup(channel, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_BITS);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void MotorDriver::writePwm(int pin, int channel, double duty) {
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  uint32_t maxVal = (1u << cfg::MOTOR_PWM_BITS) - 1;
  uint32_t v = (uint32_t)(duty * maxVal + 0.5);
#if LEDC_PIN_API
  (void)channel;
  ledcWrite(pin, v);
#else
  (void)pin;
  ledcWrite(channel, v);
#endif
}

bool MotorDriver::begin() {
  const int U = cfg::UNSET_PIN;
  switch (mode_) {
    case cfg::MotorDriverMode::UNSET:
      problem_ = "MOTOR_MODE not set";
      return false;
    case cfg::MotorDriverMode::PWM_DIR:
      if (pinA_ == U || pinB_ == U) { problem_ = "PWM_DIR needs pin A (pwm) and B (dir)"; return false; }
      pinMode(pinB_, OUTPUT);
      if (!attachPwm(pinA_, chA_)) { problem_ = "pwm attach failed"; return false; }
      break;
    case cfg::MotorDriverMode::IN1_IN2_EN:
      if (pinA_ == U || pinB_ == U || pinC_ == U) { problem_ = "IN1_IN2_EN needs pins A, B, C"; return false; }
      pinMode(pinA_, OUTPUT);
      pinMode(pinB_, OUTPUT);
      if (!attachPwm(pinC_, chC_)) { problem_ = "pwm attach failed"; return false; }
      break;
    case cfg::MotorDriverMode::DUAL_PWM:
      if (pinA_ == U || pinB_ == U) { problem_ = "DUAL_PWM needs pins A (extend) and B (retract)"; return false; }
      if (!attachPwm(pinA_, chA_) || !attachPwm(pinB_, chB_)) { problem_ = "pwm attach failed"; return false; }
      if (pinC_ != U) {
        pinMode(pinC_, OUTPUT);
        digitalWrite(pinC_, HIGH);  // enable held high
      }
      break;
  }
  ok_ = true;
  problem_ = "";
  stop();
  return true;
}

void MotorDriver::drive(int dir, double duty) {
  if (!ok_ || dir == 0) { stop(); return; }
  if (invert_) dir = -dir;
  switch (mode_) {
    case cfg::MotorDriverMode::PWM_DIR:
      digitalWrite(pinB_, dir > 0 ? HIGH : LOW);
      writePwm(pinA_, chA_, duty);
      break;
    case cfg::MotorDriverMode::IN1_IN2_EN:
      digitalWrite(pinA_, dir > 0 ? HIGH : LOW);
      digitalWrite(pinB_, dir > 0 ? LOW : HIGH);
      writePwm(pinC_, chC_, duty);
      break;
    case cfg::MotorDriverMode::DUAL_PWM:
      if (dir > 0) {
        writePwm(pinB_, chB_, 0);
        writePwm(pinA_, chA_, duty);
      } else {
        writePwm(pinA_, chA_, 0);
        writePwm(pinB_, chB_, duty);
      }
      break;
    default:
      break;
  }
}

void MotorDriver::stop() {
  if (!ok_) return;
  switch (mode_) {
    case cfg::MotorDriverMode::PWM_DIR:
      writePwm(pinA_, chA_, 0);
      break;
    case cfg::MotorDriverMode::IN1_IN2_EN:
      writePwm(pinC_, chC_, 0);
      digitalWrite(pinA_, LOW);
      digitalWrite(pinB_, LOW);
      break;
    case cfg::MotorDriverMode::DUAL_PWM:
      writePwm(pinA_, chA_, 0);
      writePwm(pinB_, chB_, 0);
      break;
    default:
      break;
  }
}
