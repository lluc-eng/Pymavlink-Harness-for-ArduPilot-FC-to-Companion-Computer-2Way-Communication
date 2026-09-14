# Pymavlink Harness for ArduPilot-based FC to Companion Computer 2-Way Communication

A small, growing set of Python scripts for controlling an ArduPilot vehicle over MAVLink, built and tested against Mission Planner SITL, aimed at eventually running on a companion computer for real autonomous missions.

## Why this exists

I've recently build from the frame up a custom 10 inch quadcopter running ArduPilot with gps-based autonomy. This included choosing, buying and assembing all the parts (FC, ESC, motors, RC receiver, GPS and compass...). Now, I'm working toward flying missions that go beyond "upload a route in Mission Planner and watch it fly". The actual goal is a companion computer (likely a Raspberry Pi) onboard, talking MAVLink directly to the flight controller, that can plan, monitor, and re-task a mission in flight without a human in the loop. Before building the actual autonomy layer, I needed to know the basics actually worked: arming, mode switches, mission upload, parameter changes, offboard positioning... all of these needed to be confirmed against a real (simulated) vehicle, not just assumed from reading the MAVLink spec.

That's what this repo is right now: the shared plumbing for talking to ArduPilot reliably through pymavlink, and a bunch of validated code I can comfortably use on the actual companion computer; plus the first two scripts that exercise it end to end against Mission Planner's SITL.

## What's here

- **`pymavlink_harness.py`** - the shared layer everything else builds on: opening a connection, arming/disarming (including a documented force-arm path), switching flight modes, uploading a mission over the real MAVLink mission protocol, tracking mission progress, and setting/reading parameters. Every one of these waits for the vehicle to actually confirm what happened (via HEARTBEAT, PARAM_VALUE, MISSION_CURRENT, etc.) instead of trusting that a command was merely accepted.
- **`test_mission_waypoints.py`** - uploads a takeoff + 4-waypoint square + RTL mission and flies it fully autonomously in AUTO mode, checking altitude and position against real telemetry at every stage, including confirming the vehicle actually lands and disarms at the end.
- **`test_guided_retask.py`** - takes a vehicle mid-mission, switches it to GUIDED, sends it a one-shot offboard position target somewhere outside the original mission, confirms it actually gets there, then switches back to AUTO and confirms the mission picks back up and finishes. This is the pattern the real companion computer would use to redirect a vehicle without aborting or re-uploading the mission.

## How it's tested

Everything runs against ArduPilot SITL, hosted through Mission Planner, with its Mavlink Mirror feature exposing a UDP endpoint (`udpin:127.0.0.1:5762`) that these scripts connect to directly. So no real hardware yet, that's deliberately later, once the logic is solid against simulation.

```bash
python test_mission_waypoints.py
python test_guided_retask.py
```

Each run appends its result (pass/fail plus the actual observed numbers -- altitudes, distances, timings) to a JSON log, not just printing a pass/fail line.

## Current capabilities

- Connect and confirm a live heartbeat before doing anything else
- Arm / disarm, confirmed via HEARTBEAT, with a documented force-arm escape hatch for scripts that need to bypass prearm checks on purpose
- Mode switches, confirmed via HEARTBEAT too
- Full mission upload over the real MAVLink mission protocol handshake
- Mission progress tracking via MISSION_CURRENT
- Parameter set/get with read-back confirmation
- Autonomous waypoint mission execution, verified against telemetry end to end
- Mid-mission GUIDED offboard retasking, with confirmed divert-and-resume

## What's next

This is still in progress. Roughly the order I'm planning to build in:

- More test scripts against the same harness: geofence behavior, RC/GPS/battery/GCS failsafes, and payload actuation (a gripper) are the next subsystems to cover, same pattern as the two scripts here
- A proper companion-computer framework on top of this harness: right now every script talks to the vehicle directly through the low-level helpers here, but the actual goal is a higher-level command layer (absolute position commands, mission management, state monitoring) that an onboard autonomy stack can call without knowing MAVLink internals at all
- Once that framework is solid against SITL, move it onto real hardware: a companion computer wired to my drone's flight controller, not just a simulated one

## Requirements

- Python 3, [`pymavlink`](https://github.com/ArduPilot/pymavlink)
- Mission Planner running ArduPilot SITL with Mavlink Mirror configured as a UDP client pointed at `127.0.0.1:5762`, write access enabled
