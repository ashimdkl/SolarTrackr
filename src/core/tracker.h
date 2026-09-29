#pragma once
// the main control logic. hardware-agnostic: everything goes through interfaces.h.
//
// one call to runCycle() is one pass of
//   WAKE -> SENSE -> COMPUTE -> DECIDE -> MOVE -> VERIFY -> (caller sleeps until next wake)
//
// control law: bang-bang with a deadband, one axis at a time, closed loop on the encoder.
#include <stdint.h>
#include "axis_geometry.h"
#include "interfaces.h"
#include "kinematics.h"
#include "tracker_config.h"
#include "types.h"

enum MoveResult {
  MOVE_NONE = 0,
  MOVE_ARRIVED,
  MOVE_ALREADY_THERE,
  MOVE_LIMIT,          // hit a soft limit before the target (not a fault, target was unreachable)
  MOVE_ABORTED,        // user abort from the console
  MOVE_STALL,          // driving but encoder not changing: jam or actuator at hard end stop
  MOVE_TIMEOUT,        // took far longer than the geometry predicts
  MOVE_OVERCURRENT,    // current spike: mechanical bind
  MOVE_HUNTING,        // kept overshooting back and forth
  MOVE_SENSOR_COMM,    // i2c read failed: wiring / cable
  MOVE_SENSOR_MAGNET,  // magnet missing / too weak / too strong
  MOVE_SENSOR_RANGE,   // reading outside physical travel: bad zero or bad sensor
  MOVE_DISABLED,       // axis resting after a fault, or locked out
  MOVE_NOT_ALLOWED     // config incomplete
};

const char* moveResultName(MoveResult r);
bool moveResultIsFault(MoveResult r);

struct MoveStats {
  MoveResult result;
  double startDeg, endDeg, targetDeg;
  double seconds, predictedSeconds;
  double energyWh;
  double peakAmps;
  bool powerMeasured;
};

enum CycleOutcome { CYCLE_DONE, CYCLE_NO_TIME };

enum TargetReason { REASON_NONE, REASON_TRACK, REASON_NIGHT, REASON_WIND };

struct AxisStatus {
  bool angleValid;
  double angleDeg;
  double targetDeg;
  uint16_t raw;
  uint8_t agc;
  uint16_t magnitude;
  int faultCount;
  uint32_t disabledUntilCycle;
  MoveResult lastResult;
};

struct TrackerStatus {
  uint32_t cycle;
  bool timeValid;
  UtcTime time;
  double lat, lng;
  bool locationFromGps;
  double sunElevDeg, sunAzDeg;
  TargetReason reason;
  bool tiltClamped, azClamped;
  AxisStatus axis[AXIS_COUNT];
  bool windValid;
  double windMps;
  bool windHold;
  double lastGainWh, lastCostWh;
  double totalMoveWh;
  uint32_t moveCount;
  double movePowerW;  // learned from the ina260
  char decision[96];
};

struct TrackerHardware {
  IAxisSensor* sensor[AXIS_COUNT];
  IAxisActuator* actuator[AXIS_COUNT];
  IPowerMonitor* power;  // may be null
  ITimeSource* time;
  IWindSensor* wind;     // may be null
  IClock* clock;
  ILogger* log;
};

class Tracker {
 public:
  Tracker(const TrackerConfig& cfg, const TrackerHardware& hw, bool motionAllowed);

  CycleOutcome runCycle();

  // closed-loop move of one axis, used by the cycle and by the console "goto" command
  MoveStats moveAxis(int axis, double targetDeg);

  void stopAll();
  void requestAbort() { abort_ = true; }
  bool motionAllowed() const { return motionAllowed_; }
  const TrackerStatus& status() const { return s_; }
  const AxisGeometry& geometry(int axis) const { return geom_[axis]; }
  void printStatus();

 private:
  bool readAxis(int axis, SensorReading& r, MoveResult& failure);
  bool inPhysicalRange(int axis, double deg) const;
  bool axisEnabled(int axis) const;
  void recordResult(int axis, const MoveStats& st);
  void setDecision(const char* fmt, ...);

  TrackerConfig cfg_;
  TrackerHardware hw_;
  bool motionAllowed_;
  volatile bool abort_;
  AxisGeometry geom_[AXIS_COUNT];
  TrackerStatus s_;
  double movePowerW_;
  uint32_t windHoldUntilMs_;
};
