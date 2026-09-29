// native unit tests for src/core.   pio test -e native
#include <math.h>
#include <unity.h>

#include "core/axis_geometry.h"
#include "core/energy_model.h"
#include "core/kinematics.h"
#include "core/sun_math.h"
#include "core/time_utils.h"
#include "core/tracker_config.h"

#include <stdio.h>

static const double LAT = 40.0, LNG = -83.0;

// unity ships with double asserts disabled, so use a small helper that prints the values
#define ASSERT_NEAR(tol, expected, actual) assertNear((tol), (expected), (actual), __LINE__)
static void assertNear(double tol, double expected, double actual, int line) {
  char msg[96];
  snprintf(msg, sizeof(msg), "expected %.6f +- %.6f, got %.6f", expected, tol, actual);
  UNITY_TEST_ASSERT(fabs(expected - actual) <= tol, line, msg);
}

void setUp() {}
void tearDown() {}

// highest elevation of the day, scanning each minute of the utc day
static double maxElevation(int y, int m, int d, int* hourOut, int* minOut) {
  double best = -90;
  for (int h = 0; h < 24; h++)
    for (int mi = 0; mi < 60; mi++) {
      SolarPosition p = getPosition(y, m, d, h, mi, 0, LAT, LNG);
      if (p.elevation > best) { best = p.elevation; *hourOut = h; *minOut = mi; }
    }
  return best;
}

// ---------------- sun_math ----------------

void test_june_solstice_noon_elevation() {
  int h, mi;
  double el = maxElevation(2026, 6, 21, &h, &mi);  // 90 - 40 + 23.44
  ASSERT_NEAR(0.6, 73.44, el);
}

void test_december_solstice_noon_elevation() {
  int h, mi;
  double el = maxElevation(2026, 12, 21, &h, &mi);  // 90 - 40 - 23.44
  ASSERT_NEAR(0.6, 26.56, el);
}

void test_solar_noon_time_matches_scan() {
  // the december issue in the old copy showed up as a ~1 h noon error, check both solstices
  int dates[2][3] = {{2026, 6, 21}, {2026, 12, 21}};
  for (int i = 0; i < 2; i++) {
    int h, mi;
    maxElevation(dates[i][0], dates[i][1], dates[i][2], &h, &mi);
    SolarTimes t = getTimes(dates[i][0], dates[i][1], dates[i][2], LAT, LNG);
    int scan = h * 60 + mi, calc = t.noonHour * 60 + t.noonMin;
    TEST_ASSERT_INT_WITHIN(3, scan, calc);
    TEST_ASSERT_TRUE(t.valid);
  }
}

void test_noon_azimuth_is_south() {
  int h, mi;
  maxElevation(2026, 9, 29, &h, &mi);
  SolarPosition p = getPosition(2026, 9, 29, h, mi, 0, LAT, LNG);
  ASSERT_NEAR(2.0, 180.0, p.azimuth);
}

void test_azimuth_always_in_range() {
  for (int h = 0; h < 24; h++) {
    SolarPosition p = getPosition(2026, 3, 20, h, 17, 0, LAT, LNG);
    TEST_ASSERT_TRUE(p.azimuth >= 0.0 && p.azimuth < 360.0);
  }
}

// ---------------- time utils ----------------

void test_epoch_round_trip() {
  UtcTime t = {2026, 9, 29, 17, 45, 12};
  int64_t e = utcToEpoch(t);
  TEST_ASSERT_TRUE(e == 1790703912LL);
  UtcTime b = epochToUtc(e + 86400 * 400);
  UtcTime c = epochToUtc(utcToEpoch(b));
  TEST_ASSERT_EQUAL_INT(b.year, c.year);
  TEST_ASSERT_EQUAL_INT(b.month, c.month);
  TEST_ASSERT_EQUAL_INT(b.day, c.day);
  TEST_ASSERT_EQUAL_INT(45, c.minute);
}

// ---------------- kinematics ----------------

void test_flat_panel_points_up() {
  Vec3 n = panelNormal(0, 0);
  ASSERT_NEAR(1e-9, 1.0, n.u);
}

void test_positive_tilt_faces_south_positive_az_faces_east() {
  Vec3 s = panelNormal(30, 0);
  TEST_ASSERT_TRUE(s.n < 0);  // south
  Vec3 e = panelNormal(0, 30);
  TEST_ASSERT_TRUE(e.e > 0);  // east
}

void test_unclamped_targets_face_the_sun() {
  JointLimits wide = {-89, 89};
  for (int el = 10; el <= 80; el += 10)
    for (int az = 60; az <= 300; az += 20) {
      Vec3 sun = sunVector(el, az);
      JointTargets t = computeTargets(sun, wide, wide);
      ASSERT_NEAR(1e-6, 1.0, incidenceCos(sun, t.tiltDeg, t.azDeg));
    }
}

void test_noon_target_matches_old_formula() {
  Vec3 sun = sunVector(26.5, 180.0);
  JointLimits wide = {-89, 89};
  JointTargets t = computeTargets(sun, wide, wide);
  ASSERT_NEAR(1e-6, 63.5, t.tiltDeg);  // 90 - elevation
  ASSERT_NEAR(1e-6, 0.0, t.azDeg);
}

void test_clamped_targets_beat_naive_clamp() {
  Vec3 sun = sunVector(20, 100);
  JointLimits tl = {-5, 60}, al = {-25, 25};
  JointTargets t = computeTargets(sun, tl, al);
  TEST_ASSERT_TRUE(t.azClamped);
  double naiveTilt = atan2(-sun.n, sun.u) * 180.0 / 3.14159265358979;
  if (naiveTilt < tl.minDeg) naiveTilt = tl.minDeg;
  if (naiveTilt > tl.maxDeg) naiveTilt = tl.maxDeg;
  double naive = incidenceCos(sun, naiveTilt, 25);
  TEST_ASSERT_TRUE(incidenceCos(sun, t.tiltDeg, t.azDeg) >= naive - 1e-9);
}

// ---------------- geometry ----------------

static AxisGeometryConfig tiltGeom() {
  AxisGeometryConfig g;
  g.aIn = 12.0;
  g.bIn = 35.15;
  g.thetaAtZeroDeg = 90;
  g.actRetractedIn = 30.0;
  g.actExtendedIn = 42.0;
  g.actSpeedInS = 0.4;
  g.extendSign = 1;
  g.softMarginDeg = 2;
  return g;
}

void test_stroke_at_zero_is_hypotenuse() {
  AxisGeometry g(tiltGeom());
  TEST_ASSERT_TRUE(g.valid());
  ASSERT_NEAR(0.001, 37.142, g.strokeForJoint(0));  // designer sketch
}

void test_joint_stroke_round_trip() {
  AxisGeometry g(tiltGeom());
  for (int j = -20; j <= 20; j += 5) {
    double back;
    g.jointForStroke(g.strokeForJoint(j), back);
    ASSERT_NEAR(1e-6, j, back);
  }
}

void test_soft_limits_inside_physical() {
  AxisGeometry g(tiltGeom());
  TEST_ASSERT_TRUE(g.minDeg() > g.physMinDeg());
  TEST_ASSERT_TRUE(g.maxDeg() < g.physMaxDeg());
  TEST_ASSERT_TRUE(g.minDeg() < 0 && g.maxDeg() > 0);
}

void test_unreachable_flat_is_rejected() {
  AxisGeometryConfig c = tiltGeom();
  c.actRetractedIn = 18.0;  // typical 12 in stroke actuator: 18..30 in, cannot reach 37.1
  c.actExtendedIn = 30.0;
  AxisGeometry g(c);
  TEST_ASSERT_FALSE(g.valid());
}

void test_extend_direction_follows_sign() {
  AxisGeometryConfig c = tiltGeom();
  AxisGeometry pos(c);
  TEST_ASSERT_EQUAL_INT(1, pos.actuatorDirFor(+3));
  TEST_ASSERT_EQUAL_INT(-1, pos.actuatorDirFor(-3));
  c.extendSign = -1;
  AxisGeometry neg(c);
  TEST_ASSERT_EQUAL_INT(-1, neg.actuatorDirFor(+3));
}

// ---------------- energy ----------------

void test_no_gain_when_already_aligned() {
  EnergyConfig e = {100, 1.2, 300};
  Vec3 sun = sunVector(45, 180);
  EnergyDecision d = evaluateMove(e, sun, 45, 45, 0, 45, 0, 10, 40);
  TEST_ASSERT_FALSE(d.worthIt);
}

void test_gain_when_far_off() {
  EnergyConfig e = {100, 1.2, 300};
  Vec3 sun = sunVector(30, 180);
  EnergyDecision d = evaluateMove(e, sun, 30, 0, 0, 60, 0, 20, 40);
  TEST_ASSERT_TRUE(d.worthIt);
  TEST_ASSERT_TRUE(d.gainWh > d.costWh);
}

// ---------------- config ----------------

void test_header_placeholders_block_motion() {
  TrackerConfig c = configFromHeader();
  ConfigCheck chk;
  validateConfig(c, chk);
  TEST_ASSERT_FALSE(chk.motionAllowed);  // until lab / install values are filled in
  TEST_ASSERT_TRUE(chk.count > 0);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_june_solstice_noon_elevation);
  RUN_TEST(test_december_solstice_noon_elevation);
  RUN_TEST(test_solar_noon_time_matches_scan);
  RUN_TEST(test_noon_azimuth_is_south);
  RUN_TEST(test_azimuth_always_in_range);
  RUN_TEST(test_epoch_round_trip);
  RUN_TEST(test_flat_panel_points_up);
  RUN_TEST(test_positive_tilt_faces_south_positive_az_faces_east);
  RUN_TEST(test_unclamped_targets_face_the_sun);
  RUN_TEST(test_noon_target_matches_old_formula);
  RUN_TEST(test_clamped_targets_beat_naive_clamp);
  RUN_TEST(test_stroke_at_zero_is_hypotenuse);
  RUN_TEST(test_joint_stroke_round_trip);
  RUN_TEST(test_soft_limits_inside_physical);
  RUN_TEST(test_unreachable_flat_is_rejected);
  RUN_TEST(test_extend_direction_follows_sign);
  RUN_TEST(test_no_gain_when_already_aligned);
  RUN_TEST(test_gain_when_far_off);
  RUN_TEST(test_header_placeholders_block_motion);
  return UNITY_END();
}
