#include "kinematics.h"
#include <math.h>

namespace {
const double kPi = 3.14159265358979323846;
const double kRad = kPi / 180.0;

double clampTo(double v, const JointLimits& lim, bool& clamped) {
  clamped = false;
  if (v < lim.minDeg) { clamped = true; return lim.minDeg; }
  if (v > lim.maxDeg) { clamped = true; return lim.maxDeg; }
  return v;
}
}  // namespace

Vec3 sunVector(double elevationDeg, double azimuthDeg) {
  double el = elevationDeg * kRad, az = azimuthDeg * kRad;
  Vec3 s;
  s.e = cos(el) * sin(az);
  s.n = cos(el) * cos(az);
  s.u = sin(el);
  return s;
}

Vec3 panelNormal(double tiltDeg, double azDeg) {
  double b = tiltDeg * kRad, p = azDeg * kRad;
  Vec3 v;
  v.e = sin(p);
  v.n = -cos(p) * sin(b);
  v.u = cos(p) * cos(b);
  return v;
}

double incidenceCos(const Vec3& sun, double tiltDeg, double azDeg) {
  Vec3 nrm = panelNormal(tiltDeg, azDeg);
  double c = nrm.e * sun.e + nrm.n * sun.n + nrm.u * sun.u;
  return c > 0 ? c : 0;
}

JointTargets computeTargets(const Vec3& sun, const JointLimits& tilt, const JointLimits& az) {
  JointTargets t;
  // ideal tilt: rotate about east-west until the normal lies in the plane containing the sun
  double tiltIdeal = atan2(-sun.n, sun.u) / kRad;
  t.tiltDeg = clampTo(tiltIdeal, tilt, t.tiltClamped);

  // for that (possibly clamped) tilt, the azimuth that maximizes n . s
  double b = t.tiltDeg * kRad;
  double k = -sin(b) * sun.n + cos(b) * sun.u;
  double azIdeal = atan2(sun.e, k) / kRad;
  t.azDeg = clampTo(azIdeal, az, t.azClamped);
  return t;
}
