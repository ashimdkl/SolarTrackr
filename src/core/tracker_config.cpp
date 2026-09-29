#include "tracker_config.h"
#include <stdarg.h>
#include <stdio.h>
#include "config.h"

TrackerConfig configFromHeader() {
  TrackerConfig c;
  c.fallbackLat = cfg::FALLBACK_LAT;
  c.fallbackLng = cfg::FALLBACK_LNG;
  c.cyclePeriodS = cfg::CYCLE_PERIOD_S;
  c.pollMs = cfg::CONTROL_POLL_MS;

  c.minTrackElevDeg = cfg::MIN_TRACK_ELEVATION_DEG;
  c.stowTiltDeg = cfg::STOW_TILT_DEG;
  c.stowAzDeg = cfg::STOW_AZ_DEG;
  c.deadbandDeg = cfg::DEADBAND_DEG;
  c.slowZoneDeg = cfg::SLOW_ZONE_DEG;
  c.dutyFull = cfg::DUTY_FULL;
  c.dutySlow = cfg::DUTY_SLOW;
  c.maxReversals = cfg::MAX_REVERSALS;

  c.stallWindowS = cfg::STALL_WINDOW_S;
  c.stallMinProgressDeg = cfg::STALL_MIN_PROGRESS_DEG;
  c.timeoutFactor = cfg::TIMEOUT_FACTOR;
  c.timeoutExtraS = cfg::TIMEOUT_EXTRA_S;
  c.overcurrentA = cfg::OVERCURRENT_A;
  c.sensorRetries = cfg::SENSOR_RETRIES;
  c.sensorRangeSlackDeg = cfg::SENSOR_RANGE_SLACK_DEG;
  c.faultCooldownCycles = cfg::FAULT_COOLDOWN_CYCLES;
  c.maxFaultsBeforeLockout = cfg::MAX_FAULTS_BEFORE_LOCKOUT;

  c.energy.panelRatedW = cfg::PANEL_RATED_W;
  c.energy.margin = cfg::ENERGY_MARGIN;
  c.energy.horizonS = cfg::GAIN_HORIZON_S;
  c.movePowerW = cfg::MOVE_POWER_W;
  c.allowMotionWithoutEnergyData = cfg::ALLOW_MOTION_WITHOUT_ENERGY_DATA;

  c.windStowMps = cfg::WIND_STOW_MPS;
  c.windReleaseS = cfg::WIND_RELEASE_S;

  AxisGeometryConfig& t = c.axis[AXIS_TILT];
  t.aIn = cfg::TILT_A_IN;
  t.bIn = cfg::TILT_B_IN;
  t.actRetractedIn = cfg::TILT_ACT_RETRACTED_IN;
  t.actExtendedIn = cfg::TILT_ACT_EXTENDED_IN;
  t.actSpeedInS = cfg::TILT_ACT_SPEED_IN_S;
  t.thetaAtZeroDeg = cfg::TILT_THETA_AT_ZERO_DEG;
  t.extendSign = cfg::TILT_EXTEND_SIGN;
  t.softMarginDeg = cfg::SOFT_LIMIT_MARGIN_DEG;

  AxisGeometryConfig& a = c.axis[AXIS_AZ];
  a.aIn = cfg::AZ_A_IN;
  a.bIn = cfg::AZ_B_IN;
  a.actRetractedIn = cfg::AZ_ACT_RETRACTED_IN;
  a.actExtendedIn = cfg::AZ_ACT_EXTENDED_IN;
  a.actSpeedInS = cfg::AZ_ACT_SPEED_IN_S;
  a.thetaAtZeroDeg = cfg::AZ_THETA_AT_ZERO_DEG;
  a.extendSign = cfg::AZ_EXTEND_SIGN;
  a.softMarginDeg = cfg::SOFT_LIMIT_MARGIN_DEG;

  c.enc[AXIS_TILT].zeroRaw = cfg::TILT_ENC_ZERO_RAW;
  c.enc[AXIS_TILT].dir = cfg::TILT_ENC_DIR;
  c.enc[AXIS_AZ].zeroRaw = cfg::AZ_ENC_ZERO_RAW;
  c.enc[AXIS_AZ].dir = cfg::AZ_ENC_DIR;
  return c;
}

void ConfigCheck::add(bool blocksMotion, const char* fmt, ...) {
  if (blocksMotion) motionAllowed = false;
  if (count >= kMaxItems) return;
  int n = snprintf(items[count], sizeof(items[count]), "%s ", blocksMotion ? "[blocks motion]" : "[warning]      ");
  va_list args;
  va_start(args, fmt);
  vsnprintf(items[count] + n, sizeof(items[count]) - n, fmt, args);
  va_end(args);
  count++;
}

void validateConfig(const TrackerConfig& c, ConfigCheck& out) {
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    AxisGeometry g(c.axis[ax]);
    if (!g.valid()) out.add(true, "%s geometry: %s", axisName(ax), g.problem());
    if (c.enc[ax].zeroRaw < 0 || c.enc[ax].zeroRaw > 4095)
      out.add(true, "%s encoder zero raw not set (console: raw, with panel flat)", axisName(ax));
    if (c.enc[ax].dir != 1 && c.enc[ax].dir != -1)
      out.add(true, "%s encoder dir not set (+1 or -1)", axisName(ax));
  }
  bool energyKnown = c.energy.panelRatedW > 0 && c.movePowerW > 0;
  if (!energyKnown) {
    if (c.allowMotionWithoutEnergyData)
      out.add(false, "energy gate bypassed (test mode), panel watts / move power unknown");
    else
      out.add(true, "energy gate needs PANEL_RATED_W and MOVE_POWER_W");
  }
  if (c.windStowMps <= 0) out.add(false, "wind stow disabled (WIND_STOW_MPS not set)");
  if (c.deadbandDeg < 0.5) out.add(false, "deadband under 0.5 deg will likely hunt");
}
