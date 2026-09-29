#pragma once
// one actuator triangle. sides a and b are fixed brackets, c is the actuator pin-to-pin,
// the included angle theta sits at the pivot.
//   theta = thetaAtZero + extendSign * joint
//   c     = sqrt(a^2 + b^2 - 2ab cos(theta))
// the encoder reads the joint angle directly, so this is only used to
//   1) turn actuator stroke limits into joint soft limits
//   2) predict how long a move should take (stall / timeout detection)

struct AxisGeometryConfig {
  double aIn, bIn;
  double actRetractedIn, actExtendedIn;  // pin-to-pin
  double actSpeedInS;
  double thetaAtZeroDeg;
  int extendSign;                         // +1: extending the actuator makes joint go positive
  double softMarginDeg;
};

class AxisGeometry {
 public:
  AxisGeometry();
  explicit AxisGeometry(const AxisGeometryConfig& cfg);

  bool valid() const { return valid_; }
  const char* problem() const { return problem_; }

  double strokeForJoint(double jointDeg) const;
  bool jointForStroke(double strokeIn, double& jointDeg) const;

  // physical travel (actuator end of stroke) and soft limits (with margin)
  double physMinDeg() const { return physMin_; }
  double physMaxDeg() const { return physMax_; }
  double minDeg() const { return softMin_; }
  double maxDeg() const { return softMax_; }
  double clamp(double jointDeg) const;

  double predictMoveSeconds(double fromDeg, double toDeg) const;

  // +1 extend / -1 retract to move the joint in the direction of `errorDeg`
  int actuatorDirFor(double errorDeg) const;

  const AxisGeometryConfig& config() const { return cfg_; }

 private:
  AxisGeometryConfig cfg_;
  bool valid_;
  char problem_[96];
  double physMin_, physMax_, softMin_, softMax_;
};
