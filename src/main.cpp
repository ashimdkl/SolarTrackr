// solar tracker firmware, esp32-devkitm-1
//
// boot: start whatever hardware is configured in config.h, print what is still missing,
// then run the tracker every CYCLE_PERIOD_S. motion stays blocked until every value it
// needs is filled in. the serial console (115200) is always available: type "help".

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "core/kinematics.h"
#include "core/sun_math.h"
#include "core/tracker.h"
#include "core/tracker_config.h"
#include "hal/arduino_platform.h"
#include "hal/encoder_as5600.h"
#include "hal/gps_time.h"
#include "hal/motor_driver.h"
#include "hal/photodiode.h"
#include "hal/power_ina260.h"
#include "hal/wind_sensor.h"

// ---------------------------------------------------------------- hardware objects

static ArduinoClock gClock;
static SerialLogger gLog;
static GpsTime gGps;
static Ina260Monitor gIna;
static PulseAnemometer gWind;
static Photodiode gPhoto;

static As5600Sensor gTiltEnc(Wire, cfg::AS5600_ADDR, cfg::TILT_ENC_ZERO_RAW, cfg::TILT_ENC_DIR);
static As5600Sensor gAzEnc(Wire1, cfg::AS5600_ADDR, cfg::AZ_ENC_ZERO_RAW, cfg::AZ_ENC_DIR);
static MotorDriver gTiltMotor(cfg::MOTOR_MODE, cfg::PIN_TILT_MOTOR_A, cfg::PIN_TILT_MOTOR_B,
                              cfg::PIN_TILT_MOTOR_C, cfg::TILT_MOTOR_INVERT);
static MotorDriver gAzMotor(cfg::MOTOR_MODE, cfg::PIN_AZ_MOTOR_A, cfg::PIN_AZ_MOTOR_B,
                            cfg::PIN_AZ_MOTOR_C, cfg::AZ_MOTOR_INVERT);

static As5600Sensor* gEnc[AXIS_COUNT] = {&gTiltEnc, &gAzEnc};
static MotorDriver* gMotor[AXIS_COUNT] = {&gTiltMotor, &gAzMotor};

static TrackerConfig gCfg;
static ConfigCheck gCheck;
static Tracker* gTracker = nullptr;

static bool gBusTilt = false, gBusAz = false;
static bool gPaused = false;
static uint32_t gNextCycleMs = 0;

// ---------------------------------------------------------------- helpers

static bool pinSet(int p) { return p != cfg::UNSET_PIN; }

// runs while the tracker waits inside a move: keep gps parsing, any keypress aborts the move
static void idleHook() {
  gGps.service();
  gWind.poll();
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    if (gTracker) gTracker->requestAbort();
    for (int ax = 0; ax < AXIS_COUNT; ax++) gMotor[ax]->stop();
    Serial.println("\n[console] key pressed, aborting move");
  }
}

static bool consoleAbortRequested() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    return true;
  }
  return false;
}

static int parseAxis(const char* s) {
  if (!s) return -1;
  if (s[0] == 't') return AXIS_TILT;
  if (s[0] == 'a') return AXIS_AZ;
  return -1;
}

static int parseDir(const char* s) {
  if (!s) return 0;
  if (strncmp(s, "ext", 3) == 0 || s[0] == '+') return 1;
  if (strncmp(s, "ret", 3) == 0 || s[0] == '-') return -1;
  return 0;
}

static void printConfigCheck() {
  if (gCheck.count == 0) {
    Serial.println("config: everything filled in");
    return;
  }
  Serial.printf("config: %d item(s), motion %s\n", gCheck.count,
                gCheck.motionAllowed ? "ALLOWED" : "BLOCKED");
  for (int i = 0; i < gCheck.count; i++) Serial.printf("  %s\n", gCheck.items[i]);
}

static void runCycleNow() {
  CycleOutcome o = gTracker->runCycle();
  uint32_t mv;
  if (gPhoto.readMilliVolts(mv)) Serial.printf("[sense] photodiode %lu mV\n", (unsigned long)mv);
  uint32_t waitS = (o == CYCLE_NO_TIME) ? cfg::NO_TIME_RETRY_S : cfg::CYCLE_PERIOD_S;
  gNextCycleMs = millis() + waitS * 1000u;
}

// ---------------------------------------------------------------- console commands

static void cmdHelp() {
  Serial.println(
      "commands\n"
      "  help                     this list\n"
      "  status                   tracker state + what config is still missing\n"
      "  raw                      encoder raw counts, magnet health (use for zero calibration)\n"
      "  i2cscan                  list devices on both i2c buses\n"
      "  sun                      sun position and joint targets right now (no motion)\n"
      "  jog <t|a> <ext|ret> <ms> drive one actuator open-loop, max 5000 ms, slow duty\n"
      "  stroke <t|a> <ext|ret>   full stroke until the actuator's limit switch cuts out,\n"
      "                           reports time, energy and current (needs ina260)\n"
      "  goto <t|a> <deg>         closed-loop move of one axis (needs full config)\n"
      "  cycle                    run one tracking cycle now\n"
      "  pause / resume           stop / restart automatic cycles\n"
      "  stop                     stop both motors\n"
      "  any key during a move aborts it");
}

static void cmdRaw() {
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    uint16_t raw, mag;
    uint8_t st, agc;
    if (!gEnc[ax]->enabled()) {
      Serial.printf("%-7s i2c bus not started (pins not set)\n", axisName(ax));
      continue;
    }
    if (!gEnc[ax]->readRaw(raw, st, agc, mag)) {
      Serial.printf("%-7s i2c read FAILED (wiring / cable / address)\n", axisName(ax));
      continue;
    }
    Serial.printf("%-7s raw %4u (%.2f deg)  magnet %s%s%s  agc %u  magnitude %u",
                  axisName(ax), raw, raw * 360.0 / 4096.0, (st & 0x20) ? "detected" : "MISSING",
                  (st & 0x10) ? " TOO WEAK" : "", (st & 0x08) ? " TOO STRONG" : "", agc, mag);
    if (gEnc[ax]->calibrated()) {
      SensorReading r = gEnc[ax]->read();
      Serial.printf("  -> joint %.2f deg", r.angleDeg);
    }
    Serial.println();
  }
  Serial.println("zero calibration: set the panel flat, copy each raw value into *_ENC_ZERO_RAW");
}

static void cmdI2cScan() {
  TwoWire* buses[2] = {&Wire, &Wire1};
  bool started[2] = {gBusTilt, gBusAz};
  const char* names[2] = {"Wire (tilt)", "Wire1 (azimuth)"};
  for (int b = 0; b < 2; b++) {
    if (!started[b]) {
      Serial.printf("%s: not started\n", names[b]);
      continue;
    }
    Serial.printf("%s:", names[b]);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      buses[b]->beginTransmission(addr);
      if (buses[b]->endTransmission() == 0) {
        Serial.printf(" 0x%02X", addr);
        found++;
      }
    }
    Serial.println(found ? "" : " nothing found");
  }
  Serial.println("expected: 0x36 as5600 on each bus, 0x40 ina260 on Wire");
}

static void cmdSun() {
  UtcTime t;
  if (!gGps.utcNow(t)) {
    Serial.printf("no gps time yet (sats %lu, chars %lu)\n", (unsigned long)gGps.satellites(),
                  (unsigned long)gGps.charsProcessed());
    return;
  }
  double lat = cfg::FALLBACK_LAT, lng = cfg::FALLBACK_LNG;
  bool fromGps = gGps.location(lat, lng);
  SolarPosition sp = getPosition(t.year, t.month, t.day, t.hour, t.minute, t.second, lat, lng);
  SolarTimes st = getTimes(t.year, t.month, t.day, lat, lng);
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d utc at %.4f %.4f (%s)\n", t.year, t.month, t.day,
                t.hour, t.minute, t.second, lat, lng, fromGps ? "gps" : "fallback");
  Serial.printf("sun el %.2f az %.2f\n", sp.elevation, sp.azimuth);
  if (st.valid)
    Serial.printf("sunrise %02d:%02d  noon %02d:%02d  sunset %02d:%02d utc\n", st.sunriseHour,
                  st.sunriseMin, st.noonHour, st.noonMin, st.sunsetHour, st.sunsetMin);
  JointLimits tl = {-89, 89}, al = {-89, 89};
  const AxisGeometry& gt = gTracker->geometry(AXIS_TILT);
  const AxisGeometry& ga = gTracker->geometry(AXIS_AZ);
  if (gt.valid()) { tl.minDeg = gt.minDeg(); tl.maxDeg = gt.maxDeg(); }
  if (ga.valid()) { al.minDeg = ga.minDeg(); al.maxDeg = ga.maxDeg(); }
  JointTargets jt = computeTargets(sunVector(sp.elevation, sp.azimuth), tl, al);
  Serial.printf("targets: tilt %.2f%s  azimuth %.2f%s%s\n", jt.tiltDeg, jt.tiltClamped ? " (clamped)" : "",
                jt.azDeg, jt.azClamped ? " (clamped)" : "",
                (gt.valid() && ga.valid()) ? "" : "  [limits unknown, using +-89]");
}

static void cmdJog(int ax, int dir, uint32_t ms) {
  if (!gMotor[ax]->configured()) {
    Serial.printf("%s motor not configured: %s\n", axisName(ax), gMotor[ax]->problem());
    return;
  }
  if (ms > 5000) ms = 5000;
  uint16_t raw0 = 0, raw1 = 0, mag;
  uint8_t st, agc;
  bool haveEnc = gEnc[ax]->readRaw(raw0, st, agc, mag);
  Serial.printf("jog %s %s for %lu ms (any key aborts)\n", axisName(ax), dir > 0 ? "extend" : "retract",
                (unsigned long)ms);
  gMotor[ax]->drive(dir, cfg::DUTY_SLOW);
  uint32_t t0 = millis();
  double peakA = 0;
  while ((uint32_t)(millis() - t0) < ms) {
    if (consoleAbortRequested()) {
      Serial.println("aborted");
      break;
    }
    PowerReading p = gIna.read();
    if (p.ok && fabs(p.amps) > peakA) peakA = fabs(p.amps);
    gGps.service();
    delay(10);
  }
  gMotor[ax]->stop();
  if (haveEnc && gEnc[ax]->readRaw(raw1, st, agc, mag)) {
    int delta = (int)raw1 - (int)raw0;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    Serial.printf("encoder raw %u -> %u (%+d counts, %+.2f deg)\n", raw0, raw1, delta, delta * 360.0 / 4096.0);
    Serial.println("tip: extend should move the joint positive (south / east). if the raw count went down,");
    Serial.println("     the encoder dir for this axis is -1.");
  }
  if (gIna.ok()) Serial.printf("peak current %.2f A\n", peakA);
}

static void cmdStroke(int ax, int dir) {
  if (!gMotor[ax]->configured()) {
    Serial.printf("%s motor not configured: %s\n", axisName(ax), gMotor[ax]->problem());
    return;
  }
  if (!gIna.ok()) {
    Serial.println("stroke needs the ina260 (it detects the limit switch cutting the current)");
    return;
  }
  Serial.printf("full stroke %s %s at full duty, any key aborts, 120 s cap\n", axisName(ax),
                dir > 0 ? "extend" : "retract");
  gMotor[ax]->drive(dir, 1.0);
  uint32_t t0 = millis(), last = t0, lastActive = t0;
  double wh = 0, peakA = 0, sumA = 0;
  uint32_t samples = 0;
  bool aborted = false;
  while (true) {
    if (consoleAbortRequested()) { aborted = true; break; }
    delay(20);
    uint32_t now = millis();
    double dt = (now - last) / 1000.0;
    last = now;
    PowerReading p = gIna.read();
    double a = fabs(p.amps);
    wh += fabs(p.watts) * dt / 3600.0;
    if (a > 0.15) {
      lastActive = now;
      sumA += a;
      samples++;
      if (a > peakA) peakA = a;
    }
    if (now - t0 > 1000 && now - lastActive > 1000) break;  // limit switch opened
    if (now - t0 > 120000) { Serial.println("120 s cap reached"); break; }
  }
  gMotor[ax]->stop();
  double secs = (lastActive - t0) / 1000.0;
  double avgA = samples ? sumA / samples : 0;
  Serial.printf("%s: moving %.2f s, %.3f Wh, avg %.2f A, peak %.2f A\n", aborted ? "ABORTED" : "done",
                secs, wh, avgA, peakA);
  if (!aborted && secs > 0) {
    Serial.printf("if the full stroke is 12 in: speed ~%.3f in/s  -> *_ACT_SPEED_IN_S\n", 12.0 / secs);
    Serial.printf("move power ~%.1f W at 12 V                   -> MOVE_POWER_W\n", avgA * 12.0);
    Serial.printf("overcurrent suggestion ~%.1f A               -> OVERCURRENT_A\n", peakA * 1.5);
  }
}

static void cmdGoto(int ax, double deg) {
  if (!gTracker->motionAllowed()) {
    Serial.println("motion blocked, config incomplete:");
    printConfigCheck();
    return;
  }
  MoveStats st = gTracker->moveAxis(ax, deg);
  Serial.printf("result %s\n", moveResultName(st.result));
}

static void handleLine(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  char* a3 = strtok(nullptr, " \t");

  if (!strcmp(cmd, "help")) cmdHelp();
  else if (!strcmp(cmd, "status")) {
    gTracker->printStatus();
    Serial.printf("gps: %s, sats %lu, chars %lu\n", gGps.everSynced() ? "synced" : "no fix yet",
                  (unsigned long)gGps.satellites(), (unsigned long)gGps.charsProcessed());
    Serial.printf("auto cycles %s\n", gPaused ? "PAUSED" : "running");
    printConfigCheck();
  } else if (!strcmp(cmd, "raw")) cmdRaw();
  else if (!strcmp(cmd, "i2cscan")) cmdI2cScan();
  else if (!strcmp(cmd, "sun")) cmdSun();
  else if (!strcmp(cmd, "jog")) {
    int ax = parseAxis(a1), dir = parseDir(a2);
    long ms = a3 ? atol(a3) : 0;
    if (ax < 0 || dir == 0 || ms <= 0) Serial.println("usage: jog <t|a> <ext|ret> <ms>");
    else cmdJog(ax, dir, (uint32_t)ms);
  } else if (!strcmp(cmd, "stroke")) {
    int ax = parseAxis(a1), dir = parseDir(a2);
    if (ax < 0 || dir == 0) Serial.println("usage: stroke <t|a> <ext|ret>");
    else cmdStroke(ax, dir);
  } else if (!strcmp(cmd, "goto")) {
    int ax = parseAxis(a1);
    if (ax < 0 || !a2) Serial.println("usage: goto <t|a> <deg>");
    else cmdGoto(ax, atof(a2));
  } else if (!strcmp(cmd, "cycle")) runCycleNow();
  else if (!strcmp(cmd, "pause")) { gPaused = true; Serial.println("auto cycles paused"); }
  else if (!strcmp(cmd, "resume")) { gPaused = false; gNextCycleMs = millis(); Serial.println("auto cycles resumed"); }
  else if (!strcmp(cmd, "stop")) { gTracker->stopAll(); Serial.println("motors stopped"); }
  else Serial.printf("unknown command '%s', type help\n", cmd);
}

static void pollConsole() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      if (len > 0) {
        Serial.printf("> %s\n", buf);
        handleLine(buf);
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== solar tracker, team 20 ===");

  // i2c buses: each as5600 is fixed at 0x36, so one bus per encoder
  if (pinSet(cfg::PIN_I2C_TILT_SDA) && pinSet(cfg::PIN_I2C_TILT_SCL)) {
    gBusTilt = Wire.begin(cfg::PIN_I2C_TILT_SDA, cfg::PIN_I2C_TILT_SCL, cfg::I2C_FREQ_HZ);
  }
  if (pinSet(cfg::PIN_I2C_AZ_SDA) && pinSet(cfg::PIN_I2C_AZ_SCL)) {
    gBusAz = Wire1.begin(cfg::PIN_I2C_AZ_SDA, cfg::PIN_I2C_AZ_SCL, cfg::I2C_FREQ_HZ);
  }
  gTiltEnc.setEnabled(gBusTilt);
  gAzEnc.setEnabled(gBusAz);

  bool gpsPins = pinSet(cfg::PIN_GPS_RX) && pinSet(cfg::PIN_GPS_TX);
  if (gpsPins) gGps.begin(Serial2, cfg::PIN_GPS_RX, cfg::PIN_GPS_TX, cfg::GPS_BAUD, cfg::GPS_HOLDOVER_S);

  bool inaOk = gBusTilt && gIna.begin(&Wire, cfg::INA260_ADDR);
  bool tiltMotorOk = gTiltMotor.begin();
  bool azMotorOk = gAzMotor.begin();
  bool windOk = gWind.begin(cfg::PIN_ANEMOMETER, cfg::WIND_HZ_TO_MPS);
  bool photoOk = gPhoto.begin(cfg::PIN_PHOTODIODE_ADC, cfg::PIN_SENSOR_LOAD_SW);

  // config + hardware checks
  gCfg = configFromHeader();
  validateConfig(gCfg, gCheck);
  if (!gBusTilt) gCheck.add(true, "tilt i2c pins not set (PIN_I2C_TILT_SDA/SCL)");
  if (!gBusAz) gCheck.add(true, "azimuth i2c pins not set (PIN_I2C_AZ_SDA/SCL)");
  if (!gpsPins) gCheck.add(true, "gps pins not set, no time = no tracking");
  if (!tiltMotorOk) gCheck.add(true, "tilt motor: %s", gTiltMotor.problem());
  if (!azMotorOk) gCheck.add(true, "azimuth motor: %s", gAzMotor.problem());
  if (!inaOk) gCheck.add(false, "ina260 not found, move energy estimated from MOVE_POWER_W");
  if (!windOk) gCheck.add(false, "anemometer not configured, no wind stow");
  if (!photoOk) gCheck.add(false, "photodiode not configured (validation only)");

  TrackerHardware hw;
  hw.sensor[AXIS_TILT] = &gTiltEnc;
  hw.sensor[AXIS_AZ] = &gAzEnc;
  hw.actuator[AXIS_TILT] = &gTiltMotor;
  hw.actuator[AXIS_AZ] = &gAzMotor;
  hw.power = inaOk ? &gIna : nullptr;
  hw.time = &gGps;
  hw.wind = windOk ? &gWind : nullptr;
  hw.clock = &gClock;
  hw.log = &gLog;
  gTracker = new Tracker(gCfg, hw, gCheck.motionAllowed);
  gClock.setIdleHook(idleHook);

  printConfigCheck();
  for (int ax = 0; ax < AXIS_COUNT; ax++) {
    const AxisGeometry& g = gTracker->geometry(ax);
    if (g.valid())
      Serial.printf("%s soft limits %.1f .. %.1f deg\n", axisName(ax), g.minDeg(), g.maxDeg());
  }
  Serial.println("type help for commands");
  gNextCycleMs = millis() + 5000;  // give the gps a moment
}

void loop() {
  gGps.service();
  gWind.poll();
  pollConsole();

  // a gust over the threshold triggers a cycle right away instead of waiting up to 5 min
  if (cfg::WIND_STOW_MPS > 0 && gWind.peekGust() >= cfg::WIND_STOW_MPS && !gTracker->status().windHold) {
    Serial.println("[wind] gust over threshold, running cycle now");
    runCycleNow();
  }

  if (!gPaused && (int32_t)(millis() - gNextCycleMs) >= 0) runCycleNow();
  delay(2);
}
