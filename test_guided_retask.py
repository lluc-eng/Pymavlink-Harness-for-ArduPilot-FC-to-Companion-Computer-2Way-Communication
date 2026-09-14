"""
test_guided_retask.py

Takes a vehicle that's mid-mission in AUTO, switches it to GUIDED, sends it
a one-shot position target somewhere outside the original mission, confirms
it actually gets there, then switches back to AUTO and confirms the
mission picks back up and still finishes. This is the "companion computer
overrides the mission mid-flight" pattern -- the same one a real onboard
autonomy stack would use to redirect a vehicle without re-uploading or
aborting the mission.

Reuses test_mission_waypoints.py's mission (build_test_mission) instead of
defining a new one -- retasking behavior doesn't depend on which mission is
running, and that mission's quirks (seq-0 substitution, mandatory takeoff-
first, mandatory RTL) are already worked out there.

Run: python test_guided_retask.py
Requires: Mission Planner running SITL, Mavlink Mirror configured per
pymavlink_harness.py's CONNECTION_STRING.

Two things this script exists specifically to pin down against real SITL
traffic, not just to demonstrate the happy path:

1. Whether a position-only SET_POSITION_TARGET_GLOBAL_INT needs to be
   resent to keep working. ArduPilot's docs (ardupilot.org/dev/docs/
   copter-commands-in-guided-mode.html) are explicit that velocity/
   acceleration/attitude targets need to be resent within GUID_TIMEOUT (3s
   default) or the vehicle stops, but don't say either way for a pure
   position target. This sends the divert target exactly once and then
   only watches telemetry -- no resend timer -- so a real run tells us
   which it is. If the vehicle stalls (DIVERT_STALL_WINDOW_S with no
   progress), it falls back to periodic resends and the result records
   that the one-shot assumption didn't hold for this version.

2. Whether the mission resumes from where it left off or restarts from the
   top after switching back to AUTO. I couldn't find this documented
   anywhere, so the script doesn't assume either way -- it just records
   whatever MISSION_CURRENT reports right after the AUTO switch, and only
   requires that the mission demonstrably continues (reaches RTL) rather
   than stalling.
"""

import time

from pymavlink import mavutil

from pymavlink_harness import (
    connect, arm, set_mode, set_param,
    upload_mission, wait_for_mission_item_reached, get_home_position,
    offset_latlon, distance_m, wait_for_armed_state,
    TestResult, log_result,
)
from test_mission_waypoints import (
    build_test_mission,
    TAKEOFF_ALT_M, WAYPOINT_ALT_M, WP_RADIUS_M_VALUE, AUTO_OPTIONS_VALUE,
    MISSION_ITEM_TIMEOUT_S,
)


# --------------------------------------------------------------------------
# Divert target
# --------------------------------------------------------------------------

# Deliberately outside the original mission's 40m square (see
# test_mission_waypoints.build_test_mission) so "the vehicle actually went
# somewhere new" is obvious from position alone, no need to check which
# mission item is active.
DIVERT_OFFSET_NORTH_M = -35.0   # south of home
DIVERT_OFFSET_EAST_M = -35.0    # west of home
DIVERT_ALT_M = WAYPOINT_ALT_M   # same altitude band as the mission -- isolates the test to lateral movement

DIVERT_POSITION_TOLERANCE_M = 5.0   # matches test_mission_waypoints' own tolerance
DIVERT_TIMEOUT_S = 60.0
# If distance-to-target hasn't improved by this much within this window,
# treat the single send as insufficient and start resending. 4s is just
# past GUID_TIMEOUT's documented 3s default -- if position targets turn out
# to share that same expiry, a 4s stall window won't misfire on a normal
# gap between sends.
DIVERT_STALL_WINDOW_S = 4.0
DIVERT_STALL_MIN_PROGRESS_M = 1.0
DIVERT_RESEND_INTERVAL_S = 1.0

# "At least one waypoint reached" before diverting: the first real waypoint
# is seq 2 (see build_test_mission), so MISSION_CURRENT advancing to 3
# means it's complete and the vehicle is genuinely mid-mission.
DIVERT_AFTER_SEQ = 3

# Position-only type_mask for SET_POSITION_TARGET_GLOBAL_INT, built from
# the named constants rather than a magic decimal. Ignores velocity,
# acceleration, yaw, and yaw rate; leaves x/y/z (bits 0-2) active, which is
# the only thing being commanded here.
POSITION_ONLY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def _send_divert_target(conn, lat, lon, alt_relative_m):
    """
    Send one SET_POSITION_TARGET_GLOBAL_INT, position-only, relative
    altitude (matches the rest of this project's altitude convention).
    """
    conn.mav.set_position_target_global_int_send(
        0,                          # time_boot_ms -- 0 means "now", ArduPilot doesn't use this for staleness here
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_ONLY_TYPE_MASK,
        int(lat * 1e7), int(lon * 1e7), alt_relative_m,
        0, 0, 0,    # vx, vy, vz -- ignored per type_mask
        0, 0, 0,    # afx, afy, afz -- ignored per type_mask
        0, 0,       # yaw, yaw_rate -- ignored per type_mask
    )


def divert_and_confirm(conn, target_lat, target_lon, target_alt_m,
                        tolerance_m=DIVERT_POSITION_TOLERANCE_M,
                        timeout_s=DIVERT_TIMEOUT_S):
    """
    Send the divert target and confirm the vehicle actually gets there --
    not just that the command was accepted, but that position telemetry
    shows it moved.

    Returns a dict of observations, including whether the single send was
    enough or periodic resends were needed -- see the module docstring for
    why that's the actual point of this function.
    """
    observed = {
        "target_latlon": [target_lat, target_lon],
        "target_alt_m": target_alt_m,
        "resends_required": False,
        "resend_count": 0,
    }

    print(f"[test] Sending divert target: lat={target_lat:.7f}, "
          f"lon={target_lon:.7f}, alt={target_alt_m}m (single send)")
    _send_divert_target(conn, target_lat, target_lon, target_alt_m)

    deadline = time.time() + timeout_s
    best_distance = None
    best_distance_time = time.time()
    last_resend = time.time()

    while time.time() < deadline:
        pos = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True,
                               timeout=max(0.0, deadline - time.time()))
        if pos is None:
            continue
        dist = distance_m(target_lat, target_lon, pos.lat / 1e7, pos.lon / 1e7)

        if best_distance is None or dist < best_distance - 1e-6:
            best_distance = dist
            best_distance_time = time.time()

        if dist <= tolerance_m:
            observed["final_distance_m"] = round(dist, 2)
            observed["reached"] = True
            print(f"[test] Divert target reached (within {tolerance_m}m), "
                  f"resends_required={observed['resends_required']} "
                  f"(resend_count={observed['resend_count']})")
            return observed

        stalled = (time.time() - best_distance_time) > DIVERT_STALL_WINDOW_S
        progressing_enough = (best_distance is not None
                               and (best_distance - dist) >= -DIVERT_STALL_MIN_PROGRESS_M)
        if stalled and (time.time() - last_resend) >= DIVERT_RESEND_INTERVAL_S:
            if not observed["resends_required"]:
                print(f"[test] No progress for {DIVERT_STALL_WINDOW_S}s "
                      f"after the single send (best distance so far: "
                      f"{best_distance:.1f}m) -- falling back to periodic "
                      f"resends. Position-only targets are NOT one-shot on "
                      f"this ArduPilot/SITL version.")
            observed["resends_required"] = True
            observed["resend_count"] += 1
            _send_divert_target(conn, target_lat, target_lon, target_alt_m)
            last_resend = time.time()

    observed["final_distance_m"] = round(best_distance, 2) if best_distance is not None else None
    observed["reached"] = False
    return observed


def run():
    conn = connect()
    result = TestResult("guided_retask")
    observed = {}

    try:
        home_lat, home_lon, home_alt = get_home_position(conn)
        items = build_test_mission(home_lat, home_lon, home_alt)
        first_wp_seq = 2
        rtl_seq = len(items) - 1

        upload_mission(conn, items)
        set_param(conn, "AUTO_OPTIONS", AUTO_OPTIONS_VALUE)
        set_param(conn, "WP_RADIUS_M", WP_RADIUS_M_VALUE)

        set_mode(conn, "GUIDED")
        arm(conn)
        set_mode(conn, "AUTO")

        # --- Confirm the mission is genuinely underway before diverting ---
        wait_for_mission_item_reached(conn, DIVERT_AFTER_SEQ, timeout=MISSION_ITEM_TIMEOUT_S)
        print(f"[test] Mission item {DIVERT_AFTER_SEQ} reached -- vehicle is "
              f"mid-mission, proceeding with GUIDED divert")
        observed["diverted_after_seq"] = DIVERT_AFTER_SEQ

        # --- Switch to GUIDED for offboard control ---
        set_mode(conn, "GUIDED")

        # --- Send the divert target, confirm the vehicle actually moves ---
        divert_lat, divert_lon = offset_latlon(
            home_lat, home_lon, DIVERT_OFFSET_NORTH_M, DIVERT_OFFSET_EAST_M
        )
        divert_observed = divert_and_confirm(conn, divert_lat, divert_lon, DIVERT_ALT_M)
        observed["divert"] = divert_observed
        if not divert_observed["reached"]:
            raise AssertionError(
                f"Vehicle did not reach the diverted target within "
                f"{DIVERT_TIMEOUT_S}s even with resends (best distance: "
                f"{divert_observed['final_distance_m']}m). GUIDED-mode "
                f"offboard commanding is not demonstrably working."
            )

        # --- Switch back to AUTO, observe (don't assume) resume behavior ---
        pre_resume_seq = None
        msg = conn.recv_match(type="MISSION_CURRENT", blocking=True, timeout=5.0)
        if msg is not None:
            pre_resume_seq = msg.seq
        set_mode(conn, "AUTO")
        post_resume_msg = conn.recv_match(type="MISSION_CURRENT", blocking=True, timeout=10.0)
        post_resume_seq = post_resume_msg.seq if post_resume_msg is not None else None
        observed["pre_resume_mission_current_seq"] = pre_resume_seq
        observed["post_resume_mission_current_seq"] = post_resume_seq
        print(f"[test] AUTO resumed -- MISSION_CURRENT before divert-exit: "
              f"{pre_resume_seq}, right after the AUTO switch: "
              f"{post_resume_seq}. Not asserting resume-vs-restart here (see "
              f"module docstring), just recording it.")

        # --- Confirm the mission still demonstrably completes afterward ---
        # Whether ArduPilot resumed from post_resume_seq or restarted from
        # item 1, both paths still have to pass through RTL to finish. And
        # RTL run as a mission item executes entirely inside AUTO mode --
        # HEARTBEAT.custom_mode never reports RTL, confirmed against live
        # SITL runs (see test_mission_waypoints.py) -- so completion is
        # confirmed the same way there: the vehicle coming home and
        # disarming after landing, not a mode change.
        rtl_landed = wait_for_armed_state(conn, want_armed=False,
                                           timeout=MISSION_ITEM_TIMEOUT_S * 2)
        observed["rtl_landed_and_disarmed"] = rtl_landed
        if not rtl_landed:
            raise AssertionError(
                "Mission did not finish (vehicle never disarmed) after "
                f"resuming AUTO post-divert within "
                f"{MISSION_ITEM_TIMEOUT_S * 2}s -- resumption may have "
                f"silently stalled rather than genuinely continuing."
            )
        print("[test] Post-divert mission completion (RTL landing + disarm) confirmed")

        result.finish(passed=True, observed=observed)
        print("[test] guided_retask: PASS")

    except Exception as e:
        result.finish(passed=False, observed=observed, notes=str(e))
        print(f"[test] guided_retask: FAIL -- {e}")
        raise
    finally:
        log_result(result)


if __name__ == "__main__":
    run()
