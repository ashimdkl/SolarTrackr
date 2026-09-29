# lab checklist, solar tracker
## 1. actuator lengths (do for BOTH actuators)
measure pin center to pin center, from the center of the mounting hole at the base to the center of the hole at the rod tip.

1. fully retract it 
2. measure retracted length
3. fully extend it
4. measure extended length
5. check: extended minus retracted should be about 12 in. 

## 2. stroke time 
1. connect the actuator straight to 12 v 
2. start fully retracted
3. start the stopwatch and connect power, stop it when the limit switch cuts out
4. write down the extend time
5. reverse the wires, time the full retract
6. do each direction twice and keep the average

speed = stroke length / time (in per second)

## 3. current draw (if using a bench supply)
- running current in the middle of the stroke 
- peak current right at the start
- note if extend and retract are different

## 4. questions for paul
- azimuth B (30.253): measured from the shaft center, or from the crossbar?
- tilt actuator base pin: is it 2.0 in up from the bottom of the mast?
- shoulder bolt: pinned to the crossbar by the through bolt block (so it turns with the crossbar)?
- extending the tilt actuator: does the panel tip south or north?
- extending the azimuth actuator: does the panel tip east or west?
- how much shaft sticks out past each pillow block? (magnet pocket + encoder plate room)

## 5. board check
1. plug in the devkitm-1
2. upload (the → arrow in the bottom bar), then serial monitor
3. confirm the banner prints and "type help for commands" shows up
4. type help, confirm the command list prints
5. if we see "Running on single core variant", screenshot it

## record sheet

item	                        tilt	        azimuth
retracted length (in)		
extended length (in)		
real stroke (in)		
extend time (s), avg		
retract time (s), avg		
speed (in/s)		
running current (a)		
peak current (a)		
flat reachable? (y/n)		


## where these go afterwards (config.h)

- lengths: `TILT_ACT_RETRACTED_IN`, `TILT_ACT_EXTENDED_IN`, `AZ_ACT_RETRACTED_IN`, `AZ_ACT_EXTENDED_IN`
- speeds: `TILT_ACT_SPEED_IN_S`, `AZ_ACT_SPEED_IN_S`
- running current x 12: `MOVE_POWER_W`
- peak current x 1.5: `OVERCURRENT_A`
- extend directions: `TILT_EXTEND_SIGN`, `AZ_EXTEND_SIGN`
- panel watts: `PANEL_RATED_W`

or just send the filled record sheet back and it gets plugged in.