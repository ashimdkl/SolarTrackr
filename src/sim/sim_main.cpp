// native simulator. runs the real Tracker (src/core) against modeled hardware for
// whole days, in a second or two on a laptop.
//
//   pio run -e sim -t exec
//
// exercises: sun math, kinematics, soft limits, deadband, energy gate, stall/timeout
// bookkeeping, and energy accounting. actuator lengths/speed are ASSUMPTIONS until the
// lab numbers are in config.h, they are printed at the top of every run.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../core/axis_geometry.h"
#include "../core/energy_model.h"
#include "../core/kinematics.h"
#include "../core/sun_math.h"
#include "../core/time_utils.h"
#include "../core/tracker.h"
#include "../core/tracker_config.h"

// ---------------------------------------------------------------- world model

struct SimAxis {
  AxisGeometry geom;
  double strokeIn;  // current actuator pin-to-pin
  int dir;
  double duty;
  double speedInS;

  void update(double dt) {
    if (dir == 0) return;
    const AxisGeometryConfig& c = geom.config();
    strokeIn += dir * speedInS * duty * dt;
    if (strokeIn < c.actRetractedIn) strokeIn = c.actRetractedIn;  // built-in limit switch
    if (strokeIn > c.actExtendedIn) strokeIn = c.actExtendedIn;
  }
  double jointDeg() const {
    double j;
    geom.jointForStroke(strokeIn, j);
    return j;
  }
};

struct SimWorld {
  uint64_t ms;
  SimAxis axis[AXIS_COUNT];
  void advance(uint32_t dtMs) {
    uint32_t left = dtMs;
    while (left > 0) {
      uint32_t step = left > 10 ? 10 : left;
      for (int ax = 0; ax < AXIS_COUNT; ax++) axis[ax].update(step / 1000.0);
      ms += step;
      left -= step;
    }
  }
  bool anyMoving() const { return axis[0].dir != 0 || axis[1].dir != 0; }
};

static SimWorld gWorld;
static int64_t gStartEpoch = 0;

// ---------------------------------------------------------------- interface impls

class SimClock : public IClock {
 public:
  uint32_t nowMs() override { return (uint32_t)gWorld.ms; }
  void sleepMs(uint32_t ms) override { gWorld.advance(ms); }
};

class SimSensor : public IAxisSensor {
 public:
  explicit SimSensor(int axis) : axis_(axis) {}
  SensorReading read() override {
    SensorReading r;
    memset(&r, 0, sizeof(r));
    r.commOk = (rand() % 1000) != 0;  // 0.1% i2c glitches, retried by the tracker
    r.magnetOk = true;
    double j = gWorld.axis[axis_].jointDeg();
    double noise = ((rand() % 1000) / 1000.0 - 0.5) * 0.1;  // +-0.05 deg
    double quant = 360.0 / 4096.0;
    r.angleDeg = floor((j + noise) / quant + 0.5) * quant;
    r.raw = (uint16_t)((int)floor(r.angleDeg / quant + 2048.5) & 0x0FFF);
    r.agc = 128;
    r.magnitude = 2000;
    return r;
  }

 private:
  int axis_;
};

class SimActuator : public IAxisActuator {
 public:
  explicit SimActuator(int axis) : axis_(axis) {}
  void drive(int dir, double duty) override {
    gWorld.axis[axis_].dir = dir;
    gWorld.axis[axis_].duty = duty;
  }
  void stop() override { gWorld.axis[axis_].dir = 0; }

 private:
  int axis_;
};

class SimPower : public IPowerMonitor {
 public:
  PowerReading read() override {
    PowerReading p;
    p.ok = true;
    p.volts = 12.8;
    double amps = 0.02;
    for (int ax = 0; ax < AXIS_COUNT; ax++)
      if (gWorld.axis[ax].dir != 0) amps += 3.0 * gWorld.axis[ax].duty + 0.5;
    p.amps = amps;
    p.watts = p.volts * amps;
    return p;
  }
};

class SimTime : public ITimeSource {
 public:
  bool utcNow(UtcTime& out) override {
    out = epochToUtc(gStartEpoch + (int64_t)(gWorld.ms / 1000));
    return true;
  }
  bool location(double& lat, double& lng) override {
    lat = 40.0;
    lng = -83.0;
    return true;
  }
};

class SimLogger : public ILogger {
 public:
  bool verbose;
  SimLogger() : verbose(false) {}
  void logf(const char* fmt, ...) override {
    if (!verbose) return;
    va_list a;
    va_start(a, fmt);
    vprintf(fmt, a);
    va_end(a);
  }
};

// ---------------------------------------------------------------- run one day

struct DayResult {
  double trackedWh, flatWh, idealWh, moveWh;
  uint32_t moves, cycles;
  int faults;
};

static TrackerConfig simConfig() {
  TrackerConfig c = configFromHeader();

  // ASSUMED until the lab numbers exist. flat sits 1.5 in above full retraction on tilt
  // (so tilt can reach deep south for winter), and mid-stroke on azimuth.
  AxisGeometry probe;
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    AxisGeometryConfig& g = c.axis[ax];
    g.extendSign = 1;
    g.actSpeedInS = 0.4;
    double a = g.aIn, b = g.bIn;
    double c0 = sqrt(a * a + b * b - 2 * a * b * cos(g.thetaAtZeroDeg * 3.14159265358979 / 180.0));
    double below = (ax == AXIS_TILT) ? 1.5 : 6.0;
    g.actRetractedIn = c0 - below;
    g.actExtendedIn = g.actRetractedIn + 12.0;
    c.enc[ax].zeroRaw = 2048;
    c.enc[ax].dir = 1;
  }
  c.energy.panelRatedW = 100.0;
  c.movePowerW = 40.0;
  c.windStowMps = 15.0;
  return c;
}

static DayResult runDay(int year, int month, int day, bool verbose, bool printHourly) {
  TrackerConfig cfg = simConfig();

  gWorld = SimWorld();
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    gWorld.axis[ax].geom = AxisGeometry(cfg.axis[ax]);
    gWorld.axis[ax].speedInS = cfg.axis[ax].actSpeedInS;
    gWorld.axis[ax].strokeIn = gWorld.axis[ax].geom.strokeForJoint(0.0);  // start flat
  }
  UtcTime start = {year, month, day, 4, 0, 0};  // 04:00 utc = midnight edt
  gStartEpoch = utcToEpoch(start);

  SimClock clock;
  SimSensor s0(AXIS_TILT), s1(AXIS_AZ);
  SimActuator a0(AXIS_TILT), a1(AXIS_AZ);
  SimPower power;
  SimTime time;
  SimLogger log;
  log.verbose = verbose;

  TrackerHardware hw;
  hw.sensor[AXIS_TILT] = &s0;
  hw.sensor[AXIS_AZ] = &s1;
  hw.actuator[AXIS_TILT] = &a0;
  hw.actuator[AXIS_AZ] = &a1;
  hw.power = &power;
  hw.time = &time;
  hw.wind = 0;
  hw.clock = &clock;
  hw.log = &log;

  ConfigCheck check;
  validateConfig(cfg, check);
  Tracker tracker(cfg, hw, check.motionAllowed);

  DayResult res;
  memset(&res, 0, sizeof(res));
  const uint64_t dayMs = 24ull * 3600 * 1000;
  const uint32_t periodMs = cfg.cyclePeriodS * 1000;
  int lastHour = -1;

  while (gWorld.ms < dayMs) {
    uint64_t cycleStart = gWorld.ms;
    tracker.runCycle();
    res.cycles++;

    // idle until next wake, sampling harvest every minute
    while (gWorld.ms < cycleStart + periodMs && gWorld.ms < dayMs) {
      UtcTime t = epochToUtc(gStartEpoch + (int64_t)(gWorld.ms / 1000));
      SolarPosition sp = getPosition(t.year, t.month, t.day, t.hour, t.minute, t.second, 40.0, -83.0);
      if (sp.elevation > 0) {
        Vec3 sun = sunVector(sp.elevation, sp.azimuth);
        double p = cfg.energy.panelRatedW * clearSkyFraction(sp.elevation) / 60.0;  // Wh per minute
        res.trackedWh += p * incidenceCos(sun, gWorld.axis[0].jointDeg(), gWorld.axis[1].jointDeg());
        res.flatWh += p * incidenceCos(sun, 0, 0);
        res.idealWh += p;
      }
      if (printHourly && t.hour != lastHour) {
        lastHour = t.hour;
        const TrackerStatus& st = tracker.status();
        printf("  %02d:00 utc  sun el %6.2f az %6.2f | tilt %6.2f -> %6.2f%s | az %6.2f -> %6.2f%s\n",
               t.hour, st.sunElevDeg, st.sunAzDeg, gWorld.axis[0].jointDeg(),
               st.axis[0].targetDeg, st.tiltClamped ? "*" : " ", gWorld.axis[1].jointDeg(),
               st.axis[1].targetDeg, st.azClamped ? "*" : " ");
      }
      gWorld.advance(60000);
    }
  }
  const TrackerStatus& st = tracker.status();
  res.moveWh = st.totalMoveWh;
  res.moves = st.moveCount;
  for (int ax = 0; ax < AXIS_COUNT; ax++) res.faults += st.axis[ax].faultCount;
  return res;
}

int main(int argc, char** argv) {
  srand(20);
  bool verbose = false;
  int y = 0, m = 0, d = 0;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) verbose = true;
    else sscanf(argv[i], "%d-%d-%d", &y, &m, &d);
  }

  TrackerConfig c = simConfig();
  printf("solar tracker sim, columbus oh (40.0, -83.0)\n");
  printf("ASSUMED (replace with lab numbers in config.h):\n");
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    AxisGeometry g(c.axis[ax]);
    printf("  %-7s a %.3f b %.3f  actuator %.2f..%.2f in @ %.2f in/s  -> soft limits %.1f..%.1f deg\n",
           axisName(ax), c.axis[ax].aIn, c.axis[ax].bIn, c.axis[ax].actRetractedIn,
           c.axis[ax].actExtendedIn, c.axis[ax].actSpeedInS, g.minDeg(), g.maxDeg());
  }
  printf("  panel %.0f W, move power %.0f W, clear sky all day\n\n", c.energy.panelRatedW, c.movePowerW);

  int dates[3][3] = {{2026, 6, 21}, {2026, 9, 29}, {2026, 12, 21}};
  int n = 3;
  if (y > 0) { dates[0][0] = y; dates[0][1] = m; dates[0][2] = d; n = 1; }

  for (int i = 0; i < n; i++) {
    printf("=== %04d-%02d-%02d ===\n", dates[i][0], dates[i][1], dates[i][2]);
    DayResult r = runDay(dates[i][0], dates[i][1], dates[i][2], verbose, true);
    printf("  harvest: tracked %.1f Wh, flat %.1f Wh, ideal %.1f Wh (tracked = %.0f%% of ideal, %+.0f%% vs flat)\n",
           r.trackedWh, r.flatWh, r.idealWh, 100.0 * r.trackedWh / r.idealWh,
           100.0 * (r.trackedWh - r.flatWh) / r.flatWh);
    printf("  moves %u, move energy %.2f Wh, net gain vs flat %.1f Wh, faults %d, cycles %u\n\n",
           r.moves, r.moveWh, r.trackedWh - r.flatWh - r.moveWh, r.faults, r.cycles);
  }
  printf("(* = target clamped at a soft limit)\n");
  return 0;
}
