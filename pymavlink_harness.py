"""
pymavlink_harness.py

Common MAVLink plumbing for ArduPilot companion-computer scripts: connecting,
arming, switching modes, uploading missions, and setting/reading parameters,
all with the "did it actually happen" confirmation built in instead of just
trusting a bare ACK.

I kept hitting the same pattern writing one-off pymavlink scripts against a
SITL instance -- connect, arm, wait for the mode to actually flip, upload a
mission by hand-rolling the request/response loop, etc. -- and copying that
code between scripts meant every copy drifted slightly and broke in its own
way. This is that logic pulled out once, so new scripts just import it.

Connects to SITL through Mission Planner's Mavlink Mirror (UDP client
pointed at 127.0.0.1:5762, write access on) -- see CONNECTION_STRING below.

Typical usage:

    from pymavlink_harness import connect, arm, set_mode, takeoff, \
        set_param, get_param, upload_mission, wait_for_mission_item_reached, \
        TestResult, log_result

    conn = connect()
    result = TestResult("my_script")
    try:
        set_mode(conn, "GUIDED")
        arm(conn)
        takeoff(conn, 10)
        ... do the actual thing ...
        result.finish(passed=True, observed={"waypoints_reached": 5})
    except Exception as e:
        result.finish(passed=False, notes=str(e))
        raise
    finally:
        log_result(result)
"""

import json
import math
import os
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from pymavlink import mavutil


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

CONNECTION_STRING = "udpin:127.0.0.1:5762"
HEARTBEAT_TIMEOUT_S = 15.0
RESULTS_LOG_PATH = "results.json"

DEFAULT_PARAM_TIMEOUT_S = 3.0
DEFAULT_PARAM_RETRIES = 3

# ArduPilot's docs (ardupilot.org/dev/docs/mavlink-arming-and-disarming.html)
# give one magic value for both force-arm and force-disarm on
# MAV_CMD_COMPONENT_ARM_DISARM.param2: 21196. There's a second value (2989)
# floating around in some tooling (MAVProxy's "arm force" uses it), tied to
# a still-open argument about which value *should* bypass which checks
# (ardupilot/ardupilot#32996) -- but current firmware accepts either, and
# 21196 is the one the official docs actually document, so that's what this
# uses. If ArduPilot ever tightens this up and 21196 stops working, 2989 is
# the fallback to try.
FORCE_MAGIC = 21196


# --------------------------------------------------------------------------
# Connection
# --------------------------------------------------------------------------

def connect(connection_string: str = CONNECTION_STRING,
            heartbeat_timeout: float = HEARTBEAT_TIMEOUT_S):
    """
    Open a MAVLink connection and block until the first heartbeat comes in.

    Raises instead of handing back a connection object that looks fine but
    has no vehicle on the other end -- everything else here assumes a live
    heartbeat stream, so it's better to fail loud now than leave a
    confusing timeout for later.
    """
    print(f"[harness] Connecting via {connection_string} ...")
    conn = mavutil.mavlink_connection(connection_string)

    msg = conn.wait_heartbeat(timeout=heartbeat_timeout)
    if msg is None:
        raise TimeoutError(
            f"No heartbeat received within {heartbeat_timeout}s on "
            f"{connection_string}. Check: is SITL running? Is Mavlink "
            f"Mirror configured as UDP Client -> 127.0.0.1:5762 with write "
            f"access enabled? Is another client already bound to this port?"
        )

    print(f"[harness] Heartbeat received from system {conn.target_system}, "
          f"component {conn.target_component}")
    return conn


# --------------------------------------------------------------------------
# Mode / arm / disarm / takeoff
# --------------------------------------------------------------------------

def set_mode(conn, mode_name: str, timeout: float = 10.0) -> None:
    """
    Switch flight mode and block until HEARTBEAT.custom_mode actually
    confirms it, rather than trusting the COMMAND_ACK alone. The heartbeat
    is the vehicle reporting its own state, so it's the strongest
    confirmation available regardless of how reliable the ack turns out
    to be.
    """
    mapping = conn.mode_mapping()
    if not mapping or mode_name not in mapping:
        available = sorted(mapping.keys()) if mapping else "none (no heartbeat yet?)"
        raise ValueError(f"Unknown mode '{mode_name}'. Available: {available}")
    mode_id = mapping[mode_name]

    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0,
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True,
                              timeout=max(0.0, deadline - time.time()))
        if hb is not None and hb.custom_mode == mode_id:
            print(f"[harness] Mode confirmed: {mode_name}")
            return
    raise TimeoutError(f"Mode did not change to {mode_name} within {timeout}s")


def wait_for_armed_state(conn, want_armed: bool, timeout: float) -> bool:
    """
    Block until HEARTBEAT reports the armed state we're waiting for.
    Returns False (doesn't raise) on timeout, so callers can decide whether
    that's actually a failure -- used both internally by arm()/disarm() and
    directly by scripts that need to confirm a vehicle landed and disarmed
    on its own (e.g. after an autonomous RTL).
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True,
                              timeout=max(0.0, deadline - time.time()))
        if hb is None:
            continue
        is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed == want_armed:
            return True
    return False


def arm(conn, force: bool = False, timeout: float = 30.0,
        poll_interval: float = 3.0) -> Dict[str, Any]:
    """
    Arm the vehicle.

    Default (force=False): sends a normal arm command on a loop and reads
    STATUSTEXT between attempts for ArduPilot's own PreArm/Arm messages.
    Never touches ARMING_CHECK -- if arming doesn't succeed in time, this
    raises with whatever PreArm reason ArduPilot last reported, instead of
    quietly working around the check that's blocking it.

    force=True sends the documented force value (param2=21196 on
    MAV_CMD_COMPONENT_ARM_DISARM, param1=1), bypassing prearm checks at the
    firmware level -- see FORCE_MAGIC above. Useful when a script genuinely
    needs to reach an armed state the checks would normally block (e.g.
    arming with a GPS failsafe deliberately triggered, to test the failsafe
    response itself), but every forced arm is printed and recorded in the
    returned dict so it can't quietly slip into a clean pass.

    Returns: {"armed": True, "forced": bool, "attempts": int,
              "last_prearm_reason": str | None}
    """
    last_prearm_reason = None
    start = time.time()
    attempt = 0

    while time.time() - start < timeout:
        attempt += 1

        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, FORCE_MAGIC if force else 0, 0, 0, 0, 0, 0,
        )

        # Not recv_match(type='COMMAND_ACK', ...) on purpose -- filtering
        # by type would silently drop any STATUSTEXT that arrives first,
        # and we'd lose the PreArm reason depending on arrival order. Look
        # at everything that comes in during this window instead.
        ack_result = None
        deadline = time.time() + poll_interval
        while time.time() < deadline:
            msg = conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "STATUSTEXT":
                text = msg.text
                if "PreArm" in text or "Arm:" in text:
                    last_prearm_reason = text
            elif (mtype == "COMMAND_ACK"
                  and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                ack_result = msg.result
                break

        if ack_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            if wait_for_armed_state(conn, want_armed=True, timeout=5.0):
                tag = " (FORCED — checks bypassed)" if force else ""
                print(f"[harness] Armed on attempt {attempt}{tag}")
                return {"armed": True, "forced": force, "attempts": attempt,
                        "last_prearm_reason": last_prearm_reason}

        # Pace retries instead of hammering the link -- a rejected ack can
        # come back in milliseconds over loopback.
        time.sleep(poll_interval)

    raise TimeoutError(
        f"Failed to arm within {timeout}s after {attempt} attempts. "
        f"Last PreArm reason reported by ArduPilot: "
        f"{last_prearm_reason or '(none captured — check the SITL/Mission Planner console directly)'}"
    )


def disarm(conn, force: bool = False, timeout: float = 10.0) -> bool:
    """
    Disarm and confirm via HEARTBEAT. Returns True once confirmed disarmed.

    force=True uses the same documented force value as arm() -- see
    FORCE_MAGIC above.
    """
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, FORCE_MAGIC if force else 0, 0, 0, 0, 0, 0,
    )
    ok = wait_for_armed_state(conn, want_armed=False, timeout=timeout)
    print(f"[harness] Disarm {'confirmed' if ok else 'NOT confirmed within timeout'}")
    return ok


def takeoff(conn, target_altitude_m: float, timeout: float = 60.0,
            alt_tolerance_m: float = 0.2) -> float:
    """
    Send MAV_CMD_NAV_TAKEOFF and poll altitude until it's within tolerance
    of the target.

    Assumes the vehicle is already armed and in a mode that accepts an
    explicit takeoff command (GUIDED). AUTO missions take off through their
    own first waypoint instead -- don't call this for those.

    Returns the last observed relative altitude in meters.
    """
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude_m,
    )

    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5.0)
    if ack is None or ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF \
            or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        got = "no ACK received" if ack is None else f"result={ack.result}"
        raise RuntimeError(f"MAV_CMD_NAV_TAKEOFF not accepted ({got})")

    deadline = time.time() + timeout
    last_alt = 0.0
    while time.time() < deadline:
        msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True,
                               timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        last_alt = msg.relative_alt / 1000.0
        if abs(last_alt - target_altitude_m) <= alt_tolerance_m:
            print(f"[harness] Takeoff reached {last_alt:.2f}m (target {target_altitude_m}m)")
            return last_alt

    raise TimeoutError(
        f"Did not reach {target_altitude_m}m within {timeout}s "
        f"(last observed altitude: {last_alt:.2f}m)"
    )


# --------------------------------------------------------------------------
# Geo helpers
# --------------------------------------------------------------------------

def offset_latlon(lat, lon, north_m, east_m):
    """
    (lat, lon) offset by north_m/east_m meters, flat-earth approximation.
    Fine at this scale (offsets of tens of meters) -- not accurate for
    offsets of kilometers+.
    """
    lat_rad = math.radians(lat)
    d_lat = north_m / 111320.0
    d_lon = east_m / (111320.0 * math.cos(lat_rad))
    return lat + d_lat, lon + d_lon


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine great-circle distance in meters."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


# --------------------------------------------------------------------------
# Home position
# --------------------------------------------------------------------------

def get_home_position(conn, timeout: float = 10.0):
    """
    Request and return the vehicle's home position as (lat, lon, alt_m).

    Uses MAV_CMD_GET_HOME_POSITION rather than just reading whatever
    GLOBAL_POSITION_INT says right now -- that breaks as soon as this gets
    called after the vehicle has already moved.
    """
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,
        0, 0, 0, 0, 0, 0, 0,
    )
    msg = conn.recv_match(type="HOME_POSITION", blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError(
            f"No HOME_POSITION received within {timeout}s after "
            f"MAV_CMD_GET_HOME_POSITION. Vehicle may not have a GPS fix yet."
        )
    lat = msg.latitude / 1e7
    lon = msg.longitude / 1e7
    alt_m = msg.altitude / 1000.0
    print(f"[harness] Home position: lat={lat:.7f}, lon={lon:.7f}, alt={alt_m:.1f}m MSL")
    return lat, lon, alt_m


# --------------------------------------------------------------------------
# Mission upload / monitoring
#
# Both functions are content-agnostic: they don't know or care what a given
# item's command/params/x/y/z actually mean. MISSION_ITEM_INT's fields
# change meaning entirely depending on the command in that item (lat/lon/
# alt for a waypoint, something else entirely for e.g. DO_GRIPPER), so
# baking item semantics into the harness would either be a no-op wrapper or
# break the moment a script needs a shape it didn't anticipate. The harness
# owns the wire protocol; scripts own what's actually in each item.
# --------------------------------------------------------------------------

def upload_mission(conn, items: List[Dict[str, Any]],
                    mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    timeout: float = 30.0) -> None:
    """
    Upload a mission via the MAVLink mission protocol handshake and block
    until ArduPilot confirms it.

    This is a pull protocol: after MISSION_COUNT announces how many items
    are coming, the vehicle requests each one by sequence number and this
    answers from `items` as requested, rather than just firing everything
    at once. That also makes it naturally idempotent -- if the vehicle
    re-requests a seq (a dropped packet on its end), we just resend that
    item, no extra bookkeeping needed.

    Args:
        conn: connection from connect().
        items: list of dicts, one per mission item, index == seq. Keys:
            "frame" (MAV_FRAME_*, almost always an _INT variant), "command"
            (MAV_CMD_*), "x", "y", "z" (meaning depends on command --
            lat*1e7/lon*1e7/alt for a waypoint, something else for other
            commands). Optional: "current" (default 0), "autocontinue"
            (default 1), "param1"-"param4" (default 0 each).
        mission_type: MAV_MISSION_TYPE_MISSION (default), _FENCE, or
            _RALLY -- same handshake serves all three, this is the only
            thing that changes between them.
        timeout: total budget for the whole handshake, not per item.

    Raises:
        RuntimeError: vehicle requested a seq outside range(len(items)), or
            MISSION_ACK came back with anything other than
            MAV_MISSION_ACCEPTED.
        TimeoutError: handshake didn't finish in time. Reports which seqs
            were sent so far, since "sent 4/5 then nothing" and "never got
            a single request" point at different problems.

    Handles both MISSION_REQUEST_INT and the legacy MISSION_REQUEST --
    always replies with MISSION_ITEM_INT either way, and prints which one
    the vehicle actually used the first time it's seen.
    """
    count = len(items)
    print(f"[harness] Uploading mission: {count} item(s), mission_type={mission_type}")
    conn.mav.mission_count_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        count=count,
        mission_type=mission_type,
    )

    sent_seqs = set()
    seen_request_type = None
    deadline = time.time() + timeout

    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "MISSION_ACK":
            # mission_type is a MAVLink2 extension field -- treat a message
            # without it as ours, since this harness never runs two mission
            # transactions at once.
            if getattr(msg, "mission_type", mission_type) != mission_type:
                continue
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"[harness] Mission upload accepted "
                      f"({len(sent_seqs)}/{count} items sent)")
                return

            raise RuntimeError(
                f"Mission upload rejected: MISSION_ACK.type={msg.type} "
                f"(look up against the MAV_MISSION_RESULT enum). "
                f"{len(sent_seqs)}/{count} items had been sent before rejection."
            )

        if mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            if getattr(msg, "mission_type", mission_type) != mission_type:
                continue
            if seen_request_type is None:
                seen_request_type = mtype
                print(f"[harness] Vehicle is requesting items via {mtype}")

            seq = msg.seq
            if seq >= count:
                raise RuntimeError(
                    f"Vehicle requested seq {seq}, but MISSION_COUNT declared "
                    f"only {count} item(s). Aborting rather than sending "
                    f"out-of-range/stale data."
                )
            item = items[seq]
            conn.mav.mission_item_int_send(
                target_system=conn.target_system,
                target_component=conn.target_component,
                seq=seq,
                frame=item["frame"],
                command=item["command"],
                current=item.get("current", 0),
                autocontinue=item.get("autocontinue", 1),
                param1=item.get("param1", 0),
                param2=item.get("param2", 0),
                param3=item.get("param3", 0),
                param4=item.get("param4", 0),
                x=item["x"],
                y=item["y"],
                z=item["z"],
                mission_type=mission_type,
            )
            sent_seqs.add(seq)

    raise TimeoutError(
        f"Mission upload not confirmed within {timeout}s. "
        f"{len(sent_seqs)}/{count} item(s) sent before timing out "
        f"(seqs sent: {sorted(sent_seqs)})."
    )


def wait_for_mission_item_reached(conn, seq: int, timeout: float = 60.0) -> bool:
    """
    Block until MISSION_CURRENT.seq == seq, or raise on timeout.

    Only reads the vehicle's own reported current-item index -- doesn't
    know or care what that item's command is. That's what makes it useful
    both for stepping through a mission in order and for confirming a
    mission resumed/advanced after something else (a mode switch, a
    reboot) interrupted it.
    """
    deadline = time.time() + timeout
    last_seen = None
    while time.time() < deadline:
        msg = conn.recv_match(type="MISSION_CURRENT", blocking=True,
                               timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        last_seen = msg.seq
        if msg.seq == seq:
            print(f"[harness] Mission item {seq} reached (MISSION_CURRENT confirmed)")
            return True

    raise TimeoutError(
        f"Mission item {seq} not reached within {timeout}s "
        f"(last observed MISSION_CURRENT.seq: {last_seen})"
    )


# --------------------------------------------------------------------------
# Parameters — set/get with confirmation, never assumed
# --------------------------------------------------------------------------

def set_param(conn, name: str, value: float,
              param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
              timeout: float = DEFAULT_PARAM_TIMEOUT_S,
              retries: int = DEFAULT_PARAM_RETRIES,
              tolerance: float = 1e-3) -> float:
    """
    Set a parameter and confirm it actually took, by reading back the
    PARAM_VALUE ArduPilot echoes after a PARAM_SET. Never assumes a write
    succeeded just because the send didn't raise -- ArduPilot silently
    ignores PARAM_SET for an unknown parameter name, so an unconfirmed
    "success" is a real failure mode here, not a hypothetical one.

    Returns the confirmed value. Raises TimeoutError if unconfirmed after
    `retries` attempts.
    """
    if len(name) > 16:
        raise ValueError(f"Parameter name '{name}' exceeds ArduPilot's 16-char limit")
    name_b = name.encode("ascii")

    for attempt in range(1, retries + 1):
        conn.mav.param_set_send(conn.target_system, conn.target_component,
                                 name_b, float(value), param_type)

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True,
                                   timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.param_id == name:
                if abs(msg.param_value - value) <= tolerance:
                    print(f"[harness] set_param({name}={value}) confirmed")
                    return msg.param_value
                print(f"[harness] set_param({name}): echoed {msg.param_value}, "
                      f"expected ~{value} (attempt {attempt}/{retries})")
                break
        else:
            print(f"[harness] set_param({name}): no PARAM_VALUE echo within "
                  f"{timeout}s (attempt {attempt}/{retries})")

    raise TimeoutError(f"set_param({name}={value}) not confirmed after {retries} attempts")


def get_param(conn, name: str, timeout: float = DEFAULT_PARAM_TIMEOUT_S,
              retries: int = DEFAULT_PARAM_RETRIES) -> float:
    """Read a parameter via PARAM_REQUEST_READ, matched by name on receipt."""
    if len(name) > 16:
        raise ValueError(f"Parameter name '{name}' exceeds ArduPilot's 16-char limit")
    name_b = name.encode("ascii")

    for attempt in range(1, retries + 1):
        conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                          name_b, -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True,
                                   timeout=max(0.0, deadline - time.time()))
            if msg is None:
                break
            if msg.param_id == name:
                return msg.param_value
        print(f"[harness] get_param({name}): no matching PARAM_VALUE within "
              f"{timeout}s (attempt {attempt}/{retries})")

    raise TimeoutError(f"get_param({name}) not received after {retries} attempts")


# --------------------------------------------------------------------------
# Result logging
# --------------------------------------------------------------------------

@dataclass
class TestResult:
    """
    One script's outcome, JSON-serializable for the results log.
    `observed` is a free-form dict for whatever numbers back up the
    pass/fail (altitudes, distances, timing, which code path ran) --
    logging real values instead of just pass/fail is the point.
    """
    test_name: str
    start_time: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    end_time: Optional[str] = None
    passed: Optional[bool] = None
    observed: Dict[str, Any] = field(default_factory=dict)
    notes: str = ""

    def finish(self, passed: bool, observed: Optional[Dict[str, Any]] = None,
               notes: str = "") -> "TestResult":
        self.end_time = datetime.now(timezone.utc).isoformat()
        self.passed = passed
        if observed:
            self.observed.update(observed)
        if notes:
            self.notes = notes
        return self


def log_result(result: TestResult, path: str = RESULTS_LOG_PATH) -> None:
    """
    Append `result` to the JSON results log (reads, appends, rewrites the
    whole file -- fine at this scale, not worth JSON-lines for a handful of
    scripts).

    If the existing file is corrupt, this doesn't silently discard it -- it
    warns and starts a fresh list, so you notice before losing old results.
    """
    entries: List[Dict[str, Any]] = []
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                entries = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print(f"[harness] WARNING: could not read existing {path} ({e}); "
                  f"starting a fresh log. Back up the old file if it had data "
                  f"you need — it will be overwritten on the next write.")
            entries = []

    entries.append(asdict(result))
    with open(path, "w") as f:
        json.dump(entries, f, indent=2)

    status = "PASS" if result.passed else "FAIL"
    print(f"[harness] Logged result: {result.test_name} -> {status}")
