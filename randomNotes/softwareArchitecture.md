# software architecture
the code is split so the brains (core) never touch hardware directly. hardware drivers live in hal, and the laptop simulator swaps in fake hardware. same brain, three ways to run it: on the esp32, in tests, in the simulator.
```
include/   settings
src/core/  the brains, plain c++, runs anywhere
src/hal/   esp32 hardware drivers
src/sim/   fake hardware for the laptop
src/main.cpp  wires it all together on the esp32
test/      automatic checks
```

each .h file says what a piece can do. its matching .cpp file is the code that does it.

## top level

**platformio.ini**
- tells platformio how to build: `devkitm1` for the board, `native` for tests, `sim` for the simulator.

**include/config.h**
- every number we can tune lives here: bracket lengths, pins, deadband, timeouts.
- blanks we still need to fill in are marked TODO, and motors won't move until they're filled.

## src/core (the brains)

**types.h**
- shared data shapes, like "a time", "an encoder reading", "a power reading".

**interfaces.h**
- the plugs core uses to talk to hardware (read encoder, drive motor, get time).
- the esp32 and the simulator each plug in their own version.

**sun_math.h / .cpp**
- takes date, time and location, gives sun elevation and azimuth.
- also gives sunrise, solar noon and sunset.

**kinematics.h / .cpp**
- turns the sun's direction into the two joint angles the panel needs (tilt and azimuth).
- if an angle is out of reach, it picks the best reachable one instead.

**axis_geometry.h / .cpp**
- the actuator triangle math (law of cosines): actuator length to angle and back.
- works out soft limits and how long a move should take.

**energy_model.h / .cpp**
- estimates energy gained by moving vs energy spent moving.
- only says "move" if the gain beats the cost (our hard requirement).

**time_utils.h / .cpp**
- converts dates to seconds and back, so gps time can keep counting between fixes.

**tracker_config.h / .cpp**
- copies config.h into a struct the tracker reads.
- checks for missing or bad values and decides if motion is allowed.

**tracker.h / .cpp**
- the main state machine: wake, read sensors, compute target, decide, move, verify.
- does the closed-loop move and catches faults (stall, timeout, overcurrent, sensor problems).

## src/hal (esp32 hardware drivers)

**arduino_platform.h / .cpp**
- the esp32 clock and serial printing that core uses.

**gps_time.h / .cpp**
- reads the neo-6m gps and gives utc time and location.
- keeps time running from the esp32's own clock if the gps drops out briefly.

**encoder_as5600.h / .cpp**
- reads a magnetic encoder over i2c and gives the joint angle.
- also reports magnet health (missing, too weak, too strong).

**motor_driver.h / .cpp**
- drives one actuator forward or back at a set speed through the motor driver.
- supports three common driver wirings, since our driver isn't confirmed yet.

**power_ina260.h**
- reads voltage, current and power from the ina260, so we know what a move really cost.

**wind_sensor.h / .cpp**
- counts anemometer pulses to get wind speed and tracks the strongest gust.

**photodiode.h**
- turns the light sensor on, reads it, turns it off. used only to check the math on clear days.

## src/main.cpp

- starts all the hardware, prints what's still missing in config, runs the tracker every 5 minutes.
- has the serial console: help, raw, jog, stroke, goto, and more, for testing by hand.

## src/sim (laptop only)

**sim_main.cpp**
- fake actuators, encoders and gps running the real tracker through full days in seconds.
- prints where the panel pointed each hour and how much energy tracking gained vs a flat panel.

## test (laptop only)

**test_core/test_main.cpp**
- automatic checks that the math is right: sun position, solar noon, joint angles, geometry, energy gate.
- run with `pio test -e native`, all should say PASS.

## how it flows every 5 minutes

1. main.cpp wakes the tracker
2. tracker reads both encoders and the gps
3. sun_math finds the sun, kinematics finds the target angles
4. energy_model decides if moving is worth it
5. tracker moves one axis at a time with motor_driver, watching the encoder until it arrives
6. tracker re-reads the encoders to confirm, logs energy used, sleeps