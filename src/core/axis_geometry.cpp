#include "axis_geometry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
const double kPi = 3.14159265358979323846;
const double kRad = kPi / 180.0;
}

AxisGeometry::AxisGeometry() : valid_(false), physMin_(0), physMax_(0), softMin_(0), softMax_(0) {
  memset(&cfg_, 0, sizeof(cfg_));
  snprintf(problem_, sizeof(problem_), "not configured");
}

AxisGeometry::AxisGeometry(const AxisGeometryConfig& cfg)
    : cfg_(cfg), valid_(false), physMin_(0), physMax_(0), softMin_(0), softMax_(0) {
  problem_[0] = 0;
  const AxisGeometryConfig& c = cfg_;
  if (c.aIn <= 0 || c.bIn <= 0) { snprintf(problem_, sizeof(problem_), "bracket a/b not set"); return; }
  if (c.actRetractedIn <= 0 || c.actExtendedIn <= 0) {
    snprintf(problem_, sizeof(problem_), "actuator retracted/extended length not set");
    return;
  }
  if (c.actExtendedIn <= c.actRetractedIn) {
    snprintf(problem_, sizeof(problem_), "actuator extended must be > retracted");
    return;
  }
  if (c.actSpeedInS <= 0) { snprintf(problem_, sizeof(problem_), "actuator speed not set"); return; }
  if (c.extendSign != 1 && c.extendSign != -1) {
    snprintf(problem_, sizeof(problem_), "extend sign not set (+1 or -1)");
    return;
  }

  // the triangle can only span |a-b| .. a+b
  double cMin = c.actRetractedIn, cMax = c.actExtendedIn;
  double geoMin = fabs(c.aIn - c.bIn) + 1e-6, geoMax = c.aIn + c.bIn - 1e-6;
  if (cMin < geoMin) cMin = geoMin;
  if (cMax > geoMax) cMax = geoMax;
  if (cMax <= cMin) {
    snprintf(problem_, sizeof(problem_), "actuator stroke does not fit this triangle");
    return;
  }

  double j1, j2;
  jointForStroke(cMin, j1);
  jointForStroke(cMax, j2);
  physMin_ = j1 < j2 ? j1 : j2;
  physMax_ = j1 < j2 ? j2 : j1;
  softMin_ = physMin_ + c.softMarginDeg;
  softMax_ = physMax_ - c.softMarginDeg;
  if (softMax_ <= softMin_) {
    snprintf(problem_, sizeof(problem_), "soft limits collapse, travel too small");
    return;
  }

  // panel-flat (joint 0) must be reachable, it is the startup / stow pose and the encoder zero
  double c0 = strokeForJoint(0.0);
  if (c0 < c.actRetractedIn || c0 > c.actExtendedIn) {
    snprintf(problem_, sizeof(problem_), "flat needs %.2f in, actuator only spans %.2f..%.2f in",
             c0, c.actRetractedIn, c.actExtendedIn);
    return;
  }
  valid_ = true;
}

double AxisGeometry::strokeForJoint(double jointDeg) const {
  double theta = (cfg_.thetaAtZeroDeg + cfg_.extendSign * jointDeg) * kRad;
  double a = cfg_.aIn, b = cfg_.bIn;
  double c2 = a * a + b * b - 2 * a * b * cos(theta);
  return c2 > 0 ? sqrt(c2) : 0;
}

bool AxisGeometry::jointForStroke(double strokeIn, double& jointDeg) const {
  double a = cfg_.aIn, b = cfg_.bIn;
  double x = (a * a + b * b - strokeIn * strokeIn) / (2 * a * b);
  bool inRange = true;
  if (x > 1) { x = 1; inRange = false; }
  if (x < -1) { x = -1; inRange = false; }
  double thetaDeg = acos(x) / kRad;
  int s = cfg_.extendSign == 0 ? 1 : cfg_.extendSign;
  jointDeg = (thetaDeg - cfg_.thetaAtZeroDeg) / s;
  return inRange;
}

double AxisGeometry::clamp(double jointDeg) const {
  if (jointDeg < softMin_) return softMin_;
  if (jointDeg > softMax_) return softMax_;
  return jointDeg;
}

double AxisGeometry::predictMoveSeconds(double fromDeg, double toDeg) const {
  if (cfg_.actSpeedInS <= 0) return 0;
  return fabs(strokeForJoint(toDeg) - strokeForJoint(fromDeg)) / cfg_.actSpeedInS;
}

int AxisGeometry::actuatorDirFor(double errorDeg) const {
  // for theta in (0, 180) stroke grows with theta, so extending moves the joint by +extendSign
  int want = errorDeg >= 0 ? 1 : -1;
  return want * (cfg_.extendSign == 0 ? 1 : cfg_.extendSign);
}
