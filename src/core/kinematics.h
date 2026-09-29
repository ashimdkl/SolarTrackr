#pragma once
// tilt-tilt kinematics for the installed orientation (crossbar / azimuth shaft north-south).
//
// frame: e = east, n = north, u = up.
// joint chain: tilt (outer, mast-top pivot, east-west axis) carries azimuth
// (inner, pillow block shaft, along the crossbar).
//   tilt    0 = flat, + = panel normal tipped toward south
//   azimuth 0 = flat, + = panel normal tipped toward east
// panel normal = ( sin(az), -cos(az) sin(tilt), cos(az) cos(tilt) )

struct Vec3 {
  double e, n, u;
};

struct JointLimits {
  double minDeg, maxDeg;
};

struct JointTargets {
  double tiltDeg, azDeg;
  bool tiltClamped, azClamped;
};

// unit vector toward the sun. azimuth clockwise from north.
Vec3 sunVector(double elevationDeg, double azimuthDeg);

Vec3 panelNormal(double tiltDeg, double azDeg);

// cosine of the angle between panel normal and sun, 0 if the sun is behind the panel
double incidenceCos(const Vec3& sun, double tiltDeg, double azDeg);

// best reachable joint angles. if tilt hits a limit, azimuth is re-optimized for
// the clamped tilt instead of clamping both blindly.
JointTargets computeTargets(const Vec3& sun, const JointLimits& tilt, const JointLimits& az);
