# solar tracker firmware, team 20

dual-axis tilt-tilt tracker on an esp32-devkitm-1. sun position is computed from gps time and location, the panel is driven by two 12 v linear actuators, and each joint angle is read directly by an as5600 encoder.

## opening the project

1. unzip anywhere.
2. vs code, platformio home, open project, pick the `solar_tracker` folder.
3. first build downloads the esp32 platform and libraries (tinygps++, adafruit ina260). takes a few minutes once.

| env | what it does | command |
| --- | --- | --- |
| devkitm1 | the real board (default) | build / upload / monitor buttons |
| native | unit tests on the laptop | `pio test -e native` |
| sim | full simulated days on the laptop | `pio run -e sim -t exec` |

native and sim need a host c++ compiler (on windows: mingw-w64 or msys2 g++ on PATH).

## layout

```
include/config.h      every constant. placeholders are tagged TODO_LAB / TODO_INSTALL / TODO_PINS / TODO_CONFIRM
src/core/             pure c++, shared by esp32, tests and sim
  sun_math            sun position (suncalc port, fixed)
  kinematics          sun direction -> tilt / azimuth joint targets
  axis_geometry       actuator triangle, soft limits, predicted move time
  energy_model        move gain vs cost (hard requirement)
  tracker             state machine WAKE > SENSE > COMPUTE > DECIDE > MOVE > VERIFY
  tracker_config      runtime copy of config.h + validation
src/hal/              esp32 drivers: as5600, gps, motor driver, ina260, anemometer, photodiode
src/main.cpp          startup, scheduling, serial console
src/sim/              native simulator
test/test_core/       unity tests
```

## axis conventions

crossbar and azimuth shaft are installed north-south.

- tilt: mast-top pivot, rotates about east-west. 0 = flat, + = panel tipped toward south.
- azimuth: pillow block shaft, rotates about north-south. 0 = flat, + = panel tipped toward east. the team calls it azimuth, mechanically it is a roll axis.

targets: tilt = atan2(-s_n, s_u), azimuth = atan2(s_e, k) for the chosen tilt. at solar noon this reduces to the old tilt = 90 - elevation, azimuth = 0.

## safety: placeholders block motion

every unfilled value is a sentinel (-1 or 0). at boot the firmware prints a list of what is missing. while anything marked `[blocks motion]` remains, it senses, computes and logs, but never drives a motor on its own. `jog` and `stroke` in the console still work once the motor driver is configured, so hardware can be brought up piece by piece.

## what to fill in, in order

lab (actuators, bench)
- `TILT_ACT_RETRACTED_IN`, `TILT_ACT_EXTENDED_IN`, `AZ_ACT_*`: pin-to-pin lengths
- `*_ACT_SPEED_IN_S`: stroke / full-stroke time at 12 v
- `MOVE_POWER_W`: 12 v x running current
- `OVERCURRENT_A`: about 1.5x the peak running current

once the esp32, driver and ina260 are wired, `stroke t ext` measures time, energy and current and prints suggested values for these.

pins and driver
- all `PIN_*` values, redone against the devkitm-1 pinout. avoid 0, 2, 5, 12, 15 for outputs. 34-39 are input only. photodiode on 32-39 (adc1).
- `MOTOR_MODE` once the driver part is confirmed (6 a actuators, an l298n class driver is too weak).

install
- `*_ENC_ZERO_RAW`: set the panel flat, run `raw`, copy the counts.
- `*_ENC_DIR`, `*_EXTEND_SIGN`: `jog` each axis and check which way things move.

team confirmations
- `PANEL_RATED_W`
- `AZ_B_IN`: is 30.253 measured from the shaft center or the crossbar?
- `WIND_STOW_MPS`, `WIND_HZ_TO_MPS`
- stow pose (defaults to flat)

## bring-up with the console (115200 baud)

`help` lists everything. useful order:

1. `i2cscan`: expect 0x36 on each bus and 0x40 on Wire.
2. `raw`: magnet detected, agc mid-range, then record the zero counts.
3. `jog t ext 500`: check direction, encoder sign and current.
4. `stroke t ext` / `stroke t ret`: time and energy for a full stroke.
5. `sun`: gps time, sun position and targets, no motion.
6. `goto t 10`: first closed-loop move once config is complete.
7. `resume`: automatic cycles every 5 minutes.

typing anything during a move aborts it.

## changes from the technical overview doc

- `getPanelTarget` removed. it assumed a panel rotating on a vertical shaft. this structure is tilt-tilt, so targets come from `kinematics.cpp`.
- sun_math fixes: no `Arduino.h`, no `PI` clash, times rounded instead of truncated, azimuth wrapped, `getTimes` matches suncalc exactly. tests confirm solar noon at both solstices.
- everything runs in utc from the gps. the dst port is not needed.
- stow defaults to flat (startup pose, least wind area).
- both encoders sit at the top of the mast (tilt on the shoulder bolt, azimuth on the shaft end), not at the base.
- the geometry check rejects configs where panel-flat is outside the actuator's stroke. if the lab lengths cannot reach c = 37.142 in (tilt) or 32.853 in (azimuth), boot will say so.

## known board caveat

some early esp32-devkitm-1 boards have a single-core chip. if the serial monitor shows "Running on single core variant of a chip, but app is built with multi-core support", that board needs a single-core build setup. newer boards are dual core and work as is.

## simulator note

the sim uses assumed actuator lengths and speed until the lab numbers exist, and prints them at the top of every run. results show the logic works, not the real travel.
