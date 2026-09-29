#pragma once
// every tunable number for the tracker lives here.
//
// anything still set to UNSET / UNSET_PIN / UNSET_SIGN / UNSET_RAW is treated as
// "not filled in yet". the firmware checks this at boot and refuses to drive a
// motor until every value it needs for motion is filled in. sensing, logging and
// the serial console still work, so partial hardware can be tested safely.
//
// tags used below
//   TODO_LAB      measure on the bench (actuators, current draw)
//   TODO_INSTALL  set once the structure is assembled (encoder zeros, signs)
//   TODO_PINS     set once the devkitm-1 pin map is redone
//   TODO_CONFIRM  value exists but needs a yes from the designer / team
//
// axis naming (tilt-tilt tracker, crossbar and azimuth shaft installed north-south)
//   tilt     mast-top pivot, rotates about an east-west axis.
//            0 = panel flat, + = panel tipped toward south. handles the seasons.
//   azimuth  pillow block shaft, rotates about the north-south axis.
//            0 = panel flat, + = panel tipped toward east. handles the daily sweep.
//            (called azimuth by the team; mechanically it is a roll axis)

#include <stdint.h>

namespace cfg {

// ---------- sentinels ----------
constexpr double UNSET      = -1.0;
constexpr int    UNSET_PIN  = -1;
constexpr int    UNSET_SIGN = 0;
constexpr int    UNSET_RAW  = -1;

// ---------- site ----------
// used only until the gps has a fix
constexpr double FALLBACK_LAT = 40.0;
constexpr double FALLBACK_LNG = -83.0;

// ---------- timing ----------
constexpr uint32_t CYCLE_PERIOD_S   = 300;   // wake every 5 minutes
constexpr uint32_t CONTROL_POLL_MS  = 50;    // encoder poll rate while moving
constexpr uint32_t NO_TIME_RETRY_S  = 10;    // retry soon if gps has no time yet
constexpr uint32_t GPS_HOLDOVER_S   = 6 * 3600; // keep time from millis() this long after last fix

// ---------- tracking ----------
constexpr double MIN_TRACK_ELEVATION_DEG = 5.0;  // below this: night stow
constexpr double STOW_TILT_DEG = 0.0;            // flat stow (lowest wind area, also startup pose)
constexpr double STOW_AZ_DEG   = 0.0;
constexpr double DEADBAND_DEG  = 1.0;            // tighter than this and the panel hunts
constexpr double SLOW_ZONE_DEG = 4.0;            // drop duty inside this error to limit coast
constexpr double DUTY_FULL     = 1.0;            // 0..1
constexpr double DUTY_SLOW     = 0.5;            // 0..1
constexpr double SOFT_LIMIT_MARGIN_DEG = 2.0;    // stay this far inside actuator end of stroke
constexpr int    MAX_REVERSALS = 3;              // more direction flips than this in one move = hunting fault

// ---------- fault detection ----------
constexpr double STALL_WINDOW_S          = 3.0;   // no progress for this long = stall
constexpr double STALL_MIN_PROGRESS_DEG  = 0.3;
constexpr double TIMEOUT_FACTOR          = 1.5;   // timeout = predicted * factor + extra
constexpr double TIMEOUT_EXTRA_S         = 5.0;
constexpr double OVERCURRENT_A           = 8.0;   // TODO_LAB: set ~1.5x the measured running current
constexpr int    SENSOR_RETRIES          = 3;     // i2c retries before calling it a comm fault
constexpr double SENSOR_RANGE_SLACK_DEG  = 5.0;   // reading this far past physical travel = bad sensor/zero
constexpr int    FAULT_COOLDOWN_CYCLES   = 12;    // axis rests 1 hour after a fault
constexpr int    MAX_FAULTS_BEFORE_LOCKOUT = 3;   // consecutive faults before axis locks out until reboot

// ---------- energy gate ----------
// a move only happens if estimated gain > estimated cost * ENERGY_MARGIN
constexpr double PANEL_RATED_W        = UNSET;  // TODO_CONFIRM: panel nameplate watts
constexpr double MOVE_POWER_W         = UNSET;  // TODO_LAB: 12 V x measured actuator running current
constexpr double ENERGY_MARGIN        = 1.2;
constexpr double GAIN_HORIZON_S       = CYCLE_PERIOD_S;
// test only. lets the tracker move with the energy gate bypassed when the two
// values above are unknown. must be false for the real install.
constexpr bool   ALLOW_MOTION_WITHOUT_ENERGY_DATA = false;

// ---------- wind ----------
constexpr double   WIND_STOW_MPS   = UNSET;  // TODO_CONFIRM: stow threshold, UNSET disables wind stow
constexpr double   WIND_HZ_TO_MPS  = UNSET;  // TODO_LAB: anemometer calibration (pulse hz -> m/s)
constexpr uint32_t WIND_RELEASE_S  = 900;    // stay stowed this long after wind drops

// ---------- tilt axis geometry (designer sketch, inches) ----------
// A = pivot to actuator rod pin (along crossbar), B = pivot to actuator base pin (down the mast)
constexpr double TILT_A_IN = 12.000;
constexpr double TILT_B_IN = 35.150;
constexpr double TILT_THETA_AT_ZERO_DEG = 90.0;   // included angle when panel is flat
constexpr double TILT_ACT_RETRACTED_IN = UNSET;   // TODO_LAB: pin-to-pin, fully retracted
constexpr double TILT_ACT_EXTENDED_IN  = UNSET;   // TODO_LAB: pin-to-pin, fully extended
constexpr double TILT_ACT_SPEED_IN_S   = UNSET;   // TODO_LAB: stroke / full stroke time at 12 V
constexpr int    TILT_EXTEND_SIGN      = UNSET_SIGN; // TODO_INSTALL: +1 if extending tips panel south, else -1

// ---------- azimuth axis geometry (designer sketch, inches) ----------
// A = shaft center to actuator rod pin, B = shaft center to actuator base pin (down the hanging post)
constexpr double AZ_A_IN = 12.810;
constexpr double AZ_B_IN = 30.253;                // TODO_CONFIRM: measured from shaft center, not crossbar?
constexpr double AZ_THETA_AT_ZERO_DEG = 90.0;
constexpr double AZ_ACT_RETRACTED_IN = UNSET;     // TODO_LAB
constexpr double AZ_ACT_EXTENDED_IN  = UNSET;     // TODO_LAB
constexpr double AZ_ACT_SPEED_IN_S   = UNSET;     // TODO_LAB
constexpr int    AZ_EXTEND_SIGN      = UNSET_SIGN; // TODO_INSTALL: +1 if extending tips panel east, else -1

// ---------- encoder calibration (as5600) ----------
// zero raw = raw count (0..4095) read with the panel flat. use the console "raw" command.
// dir = +1 if raw count increases as the joint moves positive (south / east), else -1.
constexpr int TILT_ENC_ZERO_RAW = UNSET_RAW;   // TODO_INSTALL
constexpr int TILT_ENC_DIR      = UNSET_SIGN;  // TODO_INSTALL
constexpr int AZ_ENC_ZERO_RAW   = UNSET_RAW;   // TODO_INSTALL
constexpr int AZ_ENC_DIR        = UNSET_SIGN;  // TODO_INSTALL

// ---------- motor driver ----------
// the driver part is still unconfirmed, so all three common wiring styles are supported.
//   PWM_DIR     pin A = pwm, pin B = direction
//   IN1_IN2_EN  pin A = in1, pin B = in2, pin C = enable (pwm)       (l298n style, too weak for 6 A)
//   DUAL_PWM    pin A = extend pwm, pin B = retract pwm, pin C = optional enable held high (bts7960 style)
enum class MotorDriverMode { UNSET, PWM_DIR, IN1_IN2_EN, DUAL_PWM };
constexpr MotorDriverMode MOTOR_MODE = MotorDriverMode::UNSET;  // TODO_CONFIRM
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t  MOTOR_PWM_BITS    = 8;
constexpr bool TILT_MOTOR_INVERT = false;  // flip if "extend" actually retracts
constexpr bool AZ_MOTOR_INVERT   = false;

// ---------- pins (TODO_PINS, redo against the devkitm-1 pinout) ----------
// rules: avoid strapping pins 0, 2, 5, 12, 15 for outputs. 34-39 are input only.
// the photodiode must be on an adc1 pin (32-39) because adc2 stops working with wifi on.
constexpr int PIN_I2C_TILT_SDA = UNSET_PIN;  // Wire: tilt as5600 + ina260
constexpr int PIN_I2C_TILT_SCL = UNSET_PIN;
constexpr int PIN_I2C_AZ_SDA   = UNSET_PIN;  // Wire1: azimuth as5600
constexpr int PIN_I2C_AZ_SCL   = UNSET_PIN;
constexpr uint32_t I2C_FREQ_HZ = 100000;     // long cable runs, keep it slow

constexpr int      PIN_GPS_RX = UNSET_PIN;   // esp32 rx <- neo-6m tx
constexpr int      PIN_GPS_TX = UNSET_PIN;   // esp32 tx -> neo-6m rx
constexpr uint32_t GPS_BAUD   = 9600;

constexpr int PIN_TILT_MOTOR_A = UNSET_PIN;
constexpr int PIN_TILT_MOTOR_B = UNSET_PIN;
constexpr int PIN_TILT_MOTOR_C = UNSET_PIN;  // only for IN1_IN2_EN (required) or DUAL_PWM (optional)
constexpr int PIN_AZ_MOTOR_A   = UNSET_PIN;
constexpr int PIN_AZ_MOTOR_B   = UNSET_PIN;
constexpr int PIN_AZ_MOTOR_C   = UNSET_PIN;

constexpr int PIN_PHOTODIODE_ADC  = UNSET_PIN;  // adc1 only
constexpr int PIN_SENSOR_LOAD_SW  = UNSET_PIN;  // tps22917 enable
constexpr int PIN_ANEMOMETER      = UNSET_PIN;

constexpr uint8_t INA260_ADDR = 0x40;
constexpr uint8_t AS5600_ADDR = 0x36;

}  // namespace cfg
