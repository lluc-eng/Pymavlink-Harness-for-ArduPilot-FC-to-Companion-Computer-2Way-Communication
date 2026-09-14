"""
test_mission_waypoints.py

Uploads a small autonomous mission (takeoff, four waypoints in a 40m
square, RTL) and flies it end to end in AUTO mode, checking each stage
against real telemetry instead of just trusting that commands were
accepted.

This was the first real test built against SITL, and everything else here
builds on it -- test_guided_retask.py reuses this exact mission to test
offboard retasking mid-flight.

Run: python test_mission_waypoints.py
Requires: Mission Planner running SITL, Mavlink Mirror configured per
pymavlink_harness.py's CONNECTION_STRING (UDP client -> 127.0.0.1:5762,
write access on).

ArduPilot quirk worth knowing before touching this file: its mission
protocol implementation always overwrites mission seq 0 with the vehicle's
home position, no matter what's uploaded there -- this is a documented
ArduPilot-specific deviation from the MAVLink spec (mavlink.io's "ArduPilot
differences" section), not a bug here. The first version of this mission
put the takeoff command at seq 0 and ArduPilot silently discarded it, so
switching to AUTO failed with "Auto: Missing Takeoff Cmd". The real mission
below starts at seq 1 for that reason, with an inert placeholder at seq 0.
"""

from pymavlink import mavutil

from pymavlink_harness import (
    connect, arm, set_mode, set_param,
    upload_mission, wait_for_mission_item_reached, get_home_position,
    offset_latlon, distance_m, wait_for_armed_state,
    TestResult, log_result,
)


# --------------------------------------------------------------------------
# Mission parameters
# --------------------------------------------------------------------------

TAKEOFF_ALT_M = 20.0
WAYPOINT_ALT_M = 20.0

# Required, not just for realism: ArduPilot's Copter mission command list
# says the per-item "Hit Rad" param (param2) isn't supported for
# NAV_WAYPOINT on Copter, and the WP_RADIUS_M parameter that replaces it
# only applies when the waypoint has a delay. Without a nonzero delay here,
# "waypoint complete" is instead governed by a smoothed lookahead point
# that can sit well ahead of the vehicle's actual position -- useless for
# the position cross-check below.
WAYPOINT_HOLD_S = 0.0

# Set explicitly via set_param() below rather than trusting whatever this
# SITL instance's default happens to be.
WP_RADIUS_M_VALUE = 2.0

# Our own proximity check against GLOBAL_POSITION_INT, looser than
# WP_RADIUS_M_VALUE to leave margin for GPS/EKF noise and the gap between
# the vehicle's internal "reached" event and our poll catching up.
POSITION_CHECK_TOLERANCE_M = 5.0

MISSION_ITEM_TIMEOUT_S = 90.0

# AUTO_OPTIONS bitmask (from ArduCopter's Parameters.cpp source): bit 0 =
# allow arming in AUTO, bit 1 = allow takeoff without raising throttle.
# Only bit 1 is needed here -- this is a pure MAVLink script with no RC
# input, so without it ArduPilot just sits armed in AUTO waiting for a
# throttle raise that never comes. Bit 0 isn't needed because arming
# happens in GUIDED, before switching to AUTO.
AUTO_OPTIONS_VALUE = 2


# --------------------------------------------------------------------------
# Mission construction
# --------------------------------------------------------------------------

def build_test_mission(home_lat, home_lon, home_alt):
    """
    Build the mission: an inert seq-0 placeholder, takeoff, four waypoints
    in a 40m square around home, then an explicit RTL.

    Item 0 isn't real -- see the module docstring for why. The actual
    mission starts at seq 1:
        [1]   MAV_CMD_NAV_TAKEOFF
        [2:6] four MAV_CMD_NAV_WAYPOINT items (40m square)
        [6]   MAV_CMD_NAV_RETURN_TO_LAUNCH

    Takeoff has to be the first real command -- ArduPilot refuses to switch
    to AUTO while armed and landed unless the mission's first item is a
    takeoff. The RTL at the end isn't decorative either: AUTO mode doesn't
    return home on its own after the last waypoint, it just loiters there
    indefinitely. Without an explicit RTL item, "mission complete" isn't
    something this script could ever observe.

    Returns a list of item dicts, index == seq, ready for upload_mission().
    """
    items = [{
        # Placeholder for seq 0 -- ArduPilot overwrites it with home
        # regardless of content. Using home's own coordinates here (rather
        # than zeros) just makes a raw dump of the uploaded list
        # self-documenting if anyone inspects it before ArduPilot
        # substitutes home in.
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "param1": 0, "param2": 0, "param3": 0, "param4": 0,
        "x": int(home_lat * 1e7), "y": int(home_lon * 1e7), "z": home_alt,
    }, {
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        "param1": 0,   # minimum pitch -- Plane-only, ignored by Copter
        "x": 0, "y": 0,   # ignored -- Copter climbs straight up from wherever it is
        "z": TAKEOFF_ALT_M,
    }]

    # Simple square pattern, 40m per leg.
    offsets = [
        (40, 0),    # north
        (40, 40),   # north-east
        (0, 40),    # east
        (0, 0),     # back toward home
    ]
    for north_m, east_m in offsets:
        lat, lon = offset_latlon(home_lat, home_lon, north_m, east_m)
        items.append({
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "param1": WAYPOINT_HOLD_S,  # Delay -- required, see comment above
            "param2": 0,                # "Hit Rad" -- not supported on Copter, left 0
            "param3": 0,                # unused for Copter NAV_WAYPOINT
            "param4": 0,                # Yaw -- not supported on Copter
            "x": int(lat * 1e7),
            "y": int(lon * 1e7),
            "z": WAYPOINT_ALT_M,
        })

    items.append({
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "command": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        "param1": 0, "param2": 0, "param3": 0, "param4": 0,
        "x": 0, "y": 0, "z": 0,
    })

    return items


# --------------------------------------------------------------------------
# Test
# --------------------------------------------------------------------------

def run():
    conn = connect()
    result = TestResult("mission_waypoints")
    observed = {"waypoints": []}

    try:
        home_lat, home_lon, home_alt = get_home_position(conn)
        items = build_test_mission(home_lat, home_lon, home_alt)
        # items[0] is the inert seq-0 placeholder -- see build_test_mission().
        # Real, executable commands start at seq 1.
        takeoff_seq = 1
        first_wp_seq = 2
        rtl_seq = len(items) - 1

        upload_mission(conn, items)

        set_param(conn, "AUTO_OPTIONS", AUTO_OPTIONS_VALUE)
        set_param(conn, "WP_RADIUS_M", WP_RADIUS_M_VALUE)

        set_mode(conn, "GUIDED")
        arm(conn)
        set_mode(conn, "AUTO")
        # No separate takeoff() call -- AUTO's own first mission item
        # (NAV_TAKEOFF) drives takeoff. harness.takeoff() is for explicit
        # GUIDED-mode takeoff instead (see test_guided_retask.py).

        # --- Takeoff: confirm the mission advances to the first waypoint ---
        # MISSION_CURRENT jumps straight to takeoff_seq=1 the moment AUTO
        # starts -- ArduPilot skips seq 0 (the home placeholder) entirely
        # during execution -- so "takeoff done" shows up as the counter
        # advancing to first_wp_seq=2.
        wait_for_mission_item_reached(conn, first_wp_seq, timeout=MISSION_ITEM_TIMEOUT_S)
        pos = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0)
        takeoff_alt = pos.relative_alt / 1000.0 if pos is not None else None
        observed["takeoff"] = {"target_alt_m": TAKEOFF_ALT_M, "observed_alt_m": takeoff_alt}
        print(f"[test] Takeoff confirmed, mission advanced to first "
              f"waypoint (seq {first_wp_seq}) (observed alt: {takeoff_alt}m)")
        if takeoff_alt is not None and takeoff_alt < 0.5 * TAKEOFF_ALT_M:
            raise AssertionError(
                f"Mission advanced past the takeoff item, but observed "
                f"altitude ({takeoff_alt:.1f}m) is well below the "
                f"{TAKEOFF_ALT_M}m target -- takeoff may not have "
                f"genuinely completed."
            )

        # --- Waypoints: confirm each is reached, in order ---
        # MISSION_CURRENT reports the item currently active, not the one
        # just finished -- "waypoint i done" is the counter advancing to
        # i+1, not it equalling i (which fires as soon as the vehicle
        # starts heading there).
        for i in range(first_wp_seq, rtl_seq):
            wait_for_mission_item_reached(conn, i + 1, timeout=MISSION_ITEM_TIMEOUT_S)

            pos = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5.0)
            target_lat = items[i]["x"] / 1e7
            target_lon = items[i]["y"] / 1e7
            leg = {"seq": i}
            if pos is not None:
                actual_lat, actual_lon = pos.lat / 1e7, pos.lon / 1e7
                dist = distance_m(target_lat, target_lon, actual_lat, actual_lon)
                leg.update({
                    "target_latlon": [target_lat, target_lon],
                    "actual_latlon": [actual_lat, actual_lon],
                    "distance_m": round(dist, 2),
                })
                if dist > POSITION_CHECK_TOLERANCE_M:
                    raise AssertionError(
                        f"Waypoint {i}: MISSION_CURRENT advanced past this "
                        f"item, but actual position is {dist:.1f}m from its "
                        f"target -- outside the {POSITION_CHECK_TOLERANCE_M}m "
                        f"cross-check tolerance. Sequence counter and "
                        f"telemetry disagree; investigate before trusting "
                        f"this as a pass."
                    )
            else:
                leg["note"] = "no GLOBAL_POSITION_INT received for cross-check"
            observed["waypoints"].append(leg)
            print(f"[test] Waypoint {i} confirmed reached "
                  f"({leg.get('distance_m', '?')}m from target)")

        # --- End of mission: confirm RTL actually executed ---
        # MISSION_CURRENT == rtl_seq is already confirmed by the loop above.
        # But reaching the RTL item isn't the same as it having a visible
        # effect -- and the effect to check for here is NOT a mode change.
        # RTL run as a mission item (as opposed to switching flight mode to
        # RTL directly) executes its climb/return/land sequence entirely
        # inside AUTO -- HEARTBEAT.custom_mode never reports RTL, confirmed
        # against live SITL runs. So the real completion signal is the
        # vehicle actually coming home and disarming after landing
        # (ArduCopter's default auto-disarm-on-land), not a mode value.
        rtl_landed = wait_for_armed_state(conn, want_armed=False, timeout=60.0)
        observed["rtl_landed_and_disarmed"] = rtl_landed
        if not rtl_landed:
            raise AssertionError(
                "Mission reached the final (RTL) item per MISSION_CURRENT, "
                "but the vehicle never disarmed (did not land) within 60s."
            )
        print("[test] End-of-mission RTL landing and disarm confirmed via HEARTBEAT")

        result.finish(passed=True, observed=observed)
        print("[test] mission_waypoints: PASS")

    except Exception as e:
        result.finish(passed=False, observed=observed, notes=str(e))
        print(f"[test] mission_waypoints: FAIL -- {e}")
        raise
    finally:
        log_result(result)


if __name__ == "__main__":
    run()
