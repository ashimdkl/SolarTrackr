#include "tracker.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "energy_model.h"
#include "sun_math.h"

const char* moveResultName(MoveResult r) {
  switch (r) {
    case MOVE_NONE: return "none";
    case MOVE_ARRIVED: return "arrived";
    case MOVE_ALREADY_THERE: return "already there";
    case MOVE_LIMIT: return "soft limit";
    case MOVE_ABORTED: return "aborted";
    case MOVE_STALL: return "STALL";
    case MOVE_TIMEOUT: return "TIMEOUT";
    case MOVE_OVERCURRENT: return "OVERCURRENT";
    case MOVE_HUNTING: return "HUNTING";
    case MOVE_SENSOR_COMM: return "SENSOR COMM (i2c)";
    case MOVE_SENSOR_MAGNET: return "SENSOR MAGNET";
    case MOVE_SENSOR_RANGE: return "SENSOR RANGE";
    case MOVE_DISABLED: return "axis disabled";
    case MOVE_NOT_ALLOWED: return "motion not allowed (config)";
  }
  return "?";
}

bool moveResultIsFault(MoveResult r) {
  switch (r) {
    case MOVE_STALL:
    case MOVE_TIMEOUT:
    case MOVE_OVERCURRENT:
    case MOVE_HUNTING:
    case MOVE_SENSOR_COMM:
    case MOVE_SENSOR_MAGNET:
    case MOVE_SENSOR_RANGE:
      return true;
    default:
      return false;
  }
}

static const char* reasonName(TargetReason r) {
  switch (r) {
    case REASON_TRACK: return "track";
    case REASON_NIGHT: return "night stow";
    case REASON_WIND: return "WIND STOW";
    default: return "-";
  }
}

Tracker::Tracker(const TrackerConfig& cfg, const TrackerHardware& hw, bool motionAllowed)
    : cfg_(cfg), hw_(hw), motionAllowed_(motionAllowed), abort_(false), windHoldUntilMs_(0) {
  memset(&s_, 0, sizeof(s_));
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    geom_[ax] = AxisGeometry(cfg_.axis[ax]);
    s_.axis[ax].lastResult = MOVE_NONE;
  }
  s_.lat = cfg_.fallbackLat;
  s_.lng = cfg_.fallbackLng;
  movePowerW_ = cfg_.movePowerW > 0 ? cfg_.movePowerW : 0;
  s_.movePowerW = movePowerW_;
  snprintf(s_.decision, sizeof(s_.decision), "not run yet");
}

void Tracker::setDecision(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(s_.decision, sizeof(s_.decision), fmt, args);
  va_end(args);
  hw_.log->logf("[decide] %s\n", s_.decision);
}

static const uint32_t kLockedOut = 0xFFFFFFFFu;

bool Tracker::axisEnabled(int axis) const {
  uint32_t until = s_.axis[axis].disabledUntilCycle;
  return until != kLockedOut && s_.cycle >= until;
}

bool Tracker::inPhysicalRange(int axis, double deg) const {
  if (!geom_[axis].valid()) return true;  // can't judge without geometry
  return deg >= geom_[axis].physMinDeg() - cfg_.sensorRangeSlackDeg &&
         deg <= geom_[axis].physMaxDeg() + cfg_.sensorRangeSlackDeg;
}

bool Tracker::readAxis(int axis, SensorReading& r, MoveResult& failure) {
  failure = MOVE_NONE;
  int tries = cfg_.sensorRetries < 1 ? 1 : cfg_.sensorRetries;
  for (int i = 0; i < tries; i++) {
    r = hw_.sensor[axis]->read();
    if (r.commOk) break;
    hw_.clock->sleepMs(5);
  }
  AxisStatus& a = s_.axis[axis];
  if (!r.commOk) { failure = MOVE_SENSOR_COMM; a.angleValid = false; return false; }
  a.raw = r.raw;
  a.agc = r.agc;
  a.magnitude = r.magnitude;
  if (!r.magnetOk) { failure = MOVE_SENSOR_MAGNET; a.angleValid = false; return false; }
  if (!inPhysicalRange(axis, r.angleDeg)) { failure = MOVE_SENSOR_RANGE; a.angleValid = false; return false; }
  a.angleValid = true;
  a.angleDeg = r.angleDeg;
  return true;
}

void Tracker::recordResult(int axis, const MoveStats& st) {
  AxisStatus& a = s_.axis[axis];
  a.lastResult = st.result;
  if (moveResultIsFault(st.result)) {
    a.faultCount++;
    if (a.faultCount >= cfg_.maxFaultsBeforeLockout) {
      a.disabledUntilCycle = kLockedOut;
      hw_.log->logf("[fault] %s %s, %d in a row, LOCKED OUT until reboot\n",
                    axisName(axis), moveResultName(st.result), a.faultCount);
    } else {
      a.disabledUntilCycle = s_.cycle + (uint32_t)cfg_.faultCooldownCycles;
      hw_.log->logf("[fault] %s %s, resting %d cycles\n", axisName(axis),
                    moveResultName(st.result), cfg_.faultCooldownCycles);
    }
  } else if (st.result == MOVE_ARRIVED || st.result == MOVE_ALREADY_THERE) {
    a.faultCount = 0;
  }
}

void Tracker::stopAll() {
  for (int ax = 0; ax < AXIS_COUNT; ax++) hw_.actuator[ax]->stop();
}

MoveStats Tracker::moveAxis(int axis, double targetDeg) {
  MoveStats st;
  memset(&st, 0, sizeof(st));
  st.result = MOVE_NONE;
  abort_ = false;

  if (!motionAllowed_) { st.result = MOVE_NOT_ALLOWED; return st; }
  if (!axisEnabled(axis)) { st.result = MOVE_DISABLED; return st; }

  const AxisGeometry& g = geom_[axis];
  double target = g.clamp(targetDeg);
  st.targetDeg = target;

  SensorReading r;
  MoveResult fail;
  if (!readAxis(axis, r, fail)) { st.result = fail; recordResult(axis, st); return st; }
  double angle = r.angleDeg;
  st.startDeg = st.endDeg = angle;

  if (fabs(target - angle) <= cfg_.deadbandDeg) {
    st.result = MOVE_ALREADY_THERE;
    recordResult(axis, st);
    return st;
  }

  st.predictedSeconds = g.predictMoveSeconds(angle, target);
  double timeoutS = st.predictedSeconds * cfg_.timeoutFactor + cfg_.timeoutExtraS;
  hw_.log->logf("[move] %s %.2f -> %.2f deg, predicted %.1f s\n", axisName(axis), angle, target,
                st.predictedSeconds);

  IAxisActuator* act = hw_.actuator[axis];
  IClock* clk = hw_.clock;
  uint32_t t0 = clk->nowMs();
  uint32_t lastT = t0, lastProgressT = t0;
  double progressRef = angle;
  int lastDir = 0, reversals = 0;

  while (true) {
    double err = target - angle;
    if (fabs(err) <= cfg_.deadbandDeg) { st.result = MOVE_ARRIVED; break; }
    if (abort_) { st.result = MOVE_ABORTED; break; }
    if ((err > 0 && angle >= g.maxDeg()) || (err < 0 && angle <= g.minDeg())) {
      st.result = MOVE_LIMIT;
      break;
    }

    int dir = g.actuatorDirFor(err);
    if (lastDir != 0 && dir != lastDir) {
      reversals++;
      if (reversals > cfg_.maxReversals) { st.result = MOVE_HUNTING; break; }
      act->stop();
      clk->sleepMs(200);  // let the motor spin down before reversing
      lastProgressT = clk->nowMs();
    }
    lastDir = dir;
    act->drive(dir, fabs(err) < cfg_.slowZoneDeg ? cfg_.dutySlow : cfg_.dutyFull);

    clk->sleepMs(cfg_.pollMs);
    uint32_t now = clk->nowMs();
    double dt = (now - lastT) / 1000.0;
    lastT = now;

    PowerReading p;
    p.ok = false;
    if (hw_.power) p = hw_.power->read();
    if (p.ok) {
      st.powerMeasured = true;
      st.energyWh += fabs(p.watts) * dt / 3600.0;
      double amps = fabs(p.amps);
      if (amps > st.peakAmps) st.peakAmps = amps;
      if (amps > cfg_.overcurrentA) { st.result = MOVE_OVERCURRENT; break; }
    } else {
      st.energyWh += movePowerW_ * dt / 3600.0;
    }

    if (!readAxis(axis, r, fail)) { st.result = fail; break; }
    angle = r.angleDeg;

    if (fabs(angle - progressRef) >= cfg_.stallMinProgressDeg) {
      progressRef = angle;
      lastProgressT = now;
    }
    if ((now - lastProgressT) / 1000.0 > cfg_.stallWindowS) { st.result = MOVE_STALL; break; }
    if ((now - t0) / 1000.0 > timeoutS) { st.result = MOVE_TIMEOUT; break; }
  }

  act->stop();
  st.seconds = (clk->nowMs() - t0) / 1000.0;
  st.endDeg = angle;
  s_.axis[axis].targetDeg = target;
  hw_.log->logf("[move] %s %s at %.2f deg, %.1f s (pred %.1f), %.3f Wh, peak %.2f A\n",
                axisName(axis), moveResultName(st.result), angle, st.seconds,
                st.predictedSeconds, st.energyWh, st.peakAmps);
  recordResult(axis, st);
  return st;
}

CycleOutcome Tracker::runCycle() {
  s_.cycle++;
  hw_.log->logf("\n[wake] cycle %lu\n", (unsigned long)s_.cycle);

  // ---------- SENSE ----------
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    SensorReading r;
    MoveResult fail;
    if (readAxis(ax, r, fail)) {
      hw_.log->logf("[sense] %s %.2f deg (raw %u, agc %u, mag %u)\n", axisName(ax), r.angleDeg,
                    (unsigned)r.raw, (unsigned)r.agc, (unsigned)r.magnitude);
    } else {
      hw_.log->logf("[sense] %s %s\n", axisName(ax), moveResultName(fail));
      if (motionAllowed_ && axisEnabled(ax)) {
        MoveStats st;
        memset(&st, 0, sizeof(st));
        st.result = fail;
        recordResult(ax, st);
      }
    }
  }

  s_.windValid = hw_.wind && hw_.wind->speedMps(s_.windMps);

  UtcTime t;
  if (!hw_.time->utcNow(t)) {
    s_.timeValid = false;
    setDecision("no gps time yet, skipping cycle");
    return CYCLE_NO_TIME;
  }
  s_.timeValid = true;
  s_.time = t;
  double lat, lng;
  s_.locationFromGps = hw_.time->location(lat, lng);
  if (s_.locationFromGps) { s_.lat = lat; s_.lng = lng; }

  // ---------- COMPUTE ----------
  SolarPosition sp = getPosition(t.year, t.month, t.day, t.hour, t.minute, t.second, s_.lat, s_.lng);
  s_.sunElevDeg = sp.elevation;
  s_.sunAzDeg = sp.azimuth;
  Vec3 sun = sunVector(sp.elevation, sp.azimuth);

  uint32_t nowMs = hw_.clock->nowMs();
  if (s_.windValid && cfg_.windStowMps > 0 && s_.windMps >= cfg_.windStowMps) {
    s_.windHold = true;
    windHoldUntilMs_ = nowMs + cfg_.windReleaseS * 1000u;
  } else if (s_.windHold && (int32_t)(nowMs - windHoldUntilMs_) >= 0) {
    s_.windHold = false;
  }

  double tgt[AXIS_COUNT];
  s_.tiltClamped = s_.azClamped = false;
  if (s_.windHold) {
    s_.reason = REASON_WIND;
    tgt[AXIS_TILT] = cfg_.stowTiltDeg;
    tgt[AXIS_AZ] = cfg_.stowAzDeg;
  } else if (sp.elevation < cfg_.minTrackElevDeg) {
    s_.reason = REASON_NIGHT;
    tgt[AXIS_TILT] = cfg_.stowTiltDeg;
    tgt[AXIS_AZ] = cfg_.stowAzDeg;
  } else {
    s_.reason = REASON_TRACK;
    JointLimits tl = {geom_[AXIS_TILT].minDeg(), geom_[AXIS_TILT].maxDeg()};
    JointLimits al = {geom_[AXIS_AZ].minDeg(), geom_[AXIS_AZ].maxDeg()};
    if (!geom_[AXIS_TILT].valid()) { tl.minDeg = -90; tl.maxDeg = 90; }
    if (!geom_[AXIS_AZ].valid()) { al.minDeg = -90; al.maxDeg = 90; }
    JointTargets jt = computeTargets(sun, tl, al);
    tgt[AXIS_TILT] = jt.tiltDeg;
    tgt[AXIS_AZ] = jt.azDeg;
    s_.tiltClamped = jt.tiltClamped;
    s_.azClamped = jt.azClamped;
  }
  for (int ax = 0; ax < AXIS_COUNT; ax++) s_.axis[ax].targetDeg = tgt[ax];

  hw_.log->logf("[compute] %04d-%02d-%02d %02d:%02d:%02d utc  sun el %.2f az %.2f  (%s)\n",
                t.year, t.month, t.day, t.hour, t.minute, t.second, sp.elevation, sp.azimuth,
                reasonName(s_.reason));
  hw_.log->logf("[compute] target tilt %.2f%s  azimuth %.2f%s\n", tgt[AXIS_TILT],
                s_.tiltClamped ? " (clamped)" : "", tgt[AXIS_AZ], s_.azClamped ? " (clamped)" : "");

  // ---------- DECIDE ----------
  if (!motionAllowed_) {
    setDecision("motion blocked: config incomplete (see console: status)");
    return CYCLE_DONE;
  }

  bool need[AXIS_COUNT];
  bool any = false;
  double predicted = 0;
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    const AxisStatus& a = s_.axis[ax];
    need[ax] = axisEnabled(ax) && a.angleValid && fabs(tgt[ax] - a.angleDeg) > cfg_.deadbandDeg;
    if (need[ax]) {
      any = true;
      predicted += geom_[ax].predictMoveSeconds(a.angleDeg, tgt[ax]);
    }
  }
  if (!any) {
    setDecision("idle: within deadband (or axis disabled)");
    return CYCLE_DONE;
  }

  if (s_.reason == REASON_TRACK) {
    bool energyKnown = cfg_.energy.panelRatedW > 0 && movePowerW_ > 0;
    if (energyKnown) {
      double curT = s_.axis[AXIS_TILT].angleValid ? s_.axis[AXIS_TILT].angleDeg : tgt[AXIS_TILT];
      double curA = s_.axis[AXIS_AZ].angleValid ? s_.axis[AXIS_AZ].angleDeg : tgt[AXIS_AZ];
      double newT = need[AXIS_TILT] ? tgt[AXIS_TILT] : curT;
      double newA = need[AXIS_AZ] ? tgt[AXIS_AZ] : curA;
      EnergyDecision ed = evaluateMove(cfg_.energy, sun, sp.elevation, curT, curA, newT, newA,
                                       predicted, movePowerW_);
      s_.lastGainWh = ed.gainWh;
      s_.lastCostWh = ed.costWh;
      if (!ed.worthIt) {
        setDecision("idle: gain %.3f Wh < cost %.3f Wh x %.2f", ed.gainWh, ed.costWh,
                    cfg_.energy.margin);
        return CYCLE_DONE;
      }
      setDecision("move: gain %.3f Wh > cost %.3f Wh", ed.gainWh, ed.costWh);
    } else {
      setDecision("move: energy gate bypassed (test mode)");
    }
  } else {
    setDecision("move: %s (energy gate skipped for stow)", reasonName(s_.reason));
  }

  // ---------- MOVE ----------
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    if (!need[ax]) continue;
    MoveStats st = moveAxis(ax, tgt[ax]);
    if (st.seconds > 0) {
      s_.totalMoveWh += st.energyWh;
      s_.moveCount++;
      if (st.powerMeasured && st.seconds > 1.0) {
        double measuredW = st.energyWh * 3600.0 / st.seconds;
        movePowerW_ = movePowerW_ > 0 ? 0.7 * movePowerW_ + 0.3 * measuredW : measuredW;
        s_.movePowerW = movePowerW_;
      }
    }
    if (st.result == MOVE_ABORTED) break;
  }

  // ---------- VERIFY ----------
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    SensorReading r;
    MoveResult fail;
    if (readAxis(ax, r, fail)) {
      hw_.log->logf("[verify] %s at %.2f deg (target %.2f, err %.2f)\n", axisName(ax), r.angleDeg,
                    tgt[ax], tgt[ax] - r.angleDeg);
    }
  }
  hw_.log->logf("[verify] total move energy so far %.3f Wh over %lu moves, move power ~%.1f W\n",
                s_.totalMoveWh, (unsigned long)s_.moveCount, movePowerW_);
  return CYCLE_DONE;
}

void Tracker::printStatus() {
  ILogger* L = hw_.log;
  L->logf("cycle %lu, motion %s\n", (unsigned long)s_.cycle, motionAllowed_ ? "ALLOWED" : "BLOCKED");
  if (s_.timeValid)
    L->logf("time %04d-%02d-%02d %02d:%02d:%02d utc, loc %.4f %.4f (%s)\n", s_.time.year,
            s_.time.month, s_.time.day, s_.time.hour, s_.time.minute, s_.time.second, s_.lat,
            s_.lng, s_.locationFromGps ? "gps" : "fallback");
  else
    L->logf("time: no gps time yet\n");
  L->logf("sun el %.2f az %.2f, reason %s\n", s_.sunElevDeg, s_.sunAzDeg, reasonName(s_.reason));
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    const AxisStatus& a = s_.axis[ax];
    L->logf("%-7s angle %s%.2f target %.2f  limits %.1f..%.1f  faults %d%s  last: %s\n",
            axisName(ax), a.angleValid ? "" : "(invalid) ", a.angleDeg, a.targetDeg,
            geom_[ax].minDeg(), geom_[ax].maxDeg(), a.faultCount,
            axisEnabled(ax) ? "" : " DISABLED", moveResultName(a.lastResult));
  }
  if (s_.windValid) L->logf("wind %.1f m/s%s\n", s_.windMps, s_.windHold ? " (holding stow)" : "");
  L->logf("moves %lu, total %.3f Wh, move power ~%.1f W, last gain %.3f / cost %.3f Wh\n",
          (unsigned long)s_.moveCount, s_.totalMoveWh, movePowerW_, s_.lastGainWh, s_.lastCostWh);
  L->logf("last decision: %s\n", s_.decision);
}
