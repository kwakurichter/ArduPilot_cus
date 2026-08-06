#!/usr/bin/env python3
"""
GUIDED-mode trajectory proof-of-concept for ArduPilot with optical flow + ToF.

Two approaches are implemented:

  Approach A — go_to(): position targets (SubMode::Pos)
    Send position only, block until position error and speed are below
    thresholds.  Simple and reliable for point-to-point tasks, but motion
    between waypoints is abrupt because pos_control uses its internal speed
    limits to get there as fast as it can.

  Approach B — stream_pva(): PVA trajectory streaming (SubMode::PosVelAccel)
    Precompute position + velocity + acceleration at each time step and stream
    at 50 Hz.  Velocity and acceleration are fed forward directly into
    pos_control, giving smooth and predictable motion.  Position acts as a
    correction term to prevent drift.  GUID_TIMEOUT safety still applies:
    the drone will hover if streaming stops for longer than the timeout.

NED convention (z positive = down):
  - North  →  +x
  - East   →  +y
  - Up     →  -z  (e.g. 1 m altitude = z = -1.0)

Dependencies: pymavlink, numpy
  pip install pymavlink numpy
"""

import math
import time
import threading
import numpy as np
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# SET_POSITION_TARGET_LOCAL_NED type_mask
#
# Each bit set means "ignore this field".  Routing inside ArduPilot
# (GCS_Mavlink.cpp) checks which groups are ignored to choose the sub-mode:
#
#   pos ignored, vel used               → SubMode::VelAccel
#   pos used,    vel ignored, acc ign.  → SubMode::Pos          (no timeout)
#   pos used,    vel used               → SubMode::PosVelAccel  (GUID_TIMEOUT)
# ---------------------------------------------------------------------------
_IGN_VX, _IGN_VY, _IGN_VZ = (1 << 3), (1 << 4), (1 << 5)
_IGN_AX, _IGN_AY, _IGN_AZ = (1 << 6), (1 << 7), (1 << 8)
_IGN_YAW = 1 << 10
_IGN_YR  = 1 << 11

# Position only  → SubMode::Pos (no timeout, indefinite hold)
MASK_POS = _IGN_VX | _IGN_VY | _IGN_VZ | _IGN_AX | _IGN_AY | _IGN_AZ | _IGN_YAW | _IGN_YR

# Position + velocity + acceleration  → SubMode::PosVelAccel
MASK_PVA = _IGN_YAW | _IGN_YR


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class GuidedController:
    """
    Sends MAVLink trajectory commands to ArduPilot in GUIDED mode.

    Assumes the vehicle is already armed, in GUIDED mode, and hovering
    at the desired starting altitude before any trajectory method is called.

    If FS_GCS_ENABLE is set to anything other than 0 (disabled), the
    heartbeat thread below keeps the GCS failsafe from triggering.
    """

    def __init__(self, connection: str = 'udpin:0.0.0.0:14550',
                 heartbeat_hz: float = 2.0):
        self.mav = mavutil.mavlink_connection(connection)
        print("Waiting for heartbeat…")
        self.mav.wait_heartbeat()
        print(f"Connected — system {self.mav.target_system}, "
              f"component {self.mav.target_component}")

        self._request_streams()

        # Heartbeat thread prevents GCS failsafe if FS_GCS_ENABLE != 0
        self._hb_interval = 1.0 / heartbeat_hz
        self._stop = threading.Event()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb_thread.start()

    def close(self):
        self._stop.set()
        self.mav.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _heartbeat_loop(self):
        while not self._stop.wait(self._hb_interval):
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

    def _request_streams(self):
        """Request LOCAL_POSITION_NED at 50 Hz."""
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            50, 1
        )

    def _send_pos_target(self, x: float, y: float, z: float,
                          vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                          ax: float = 0.0, ay: float = 0.0, az: float = 0.0,
                          mask: int = MASK_POS) -> None:
        """
        SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_LOCAL_NED.
        x, y, z  : metres NED relative to EKF origin  (z negative = altitude)
        vx/vy/vz : m/s NED
        ax/ay/az : m/s² NED
        """
        self.mav.mav.set_position_target_local_ned_send(
            0,                                    # time_boot_ms (ignored)
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            x, y, z,
            vx, vy, vz,
            ax, ay, az,
            0.0, 0.0                              # yaw, yaw_rate (ignored)
        )

    def local_ned(self):
        """
        Returns the most recent (x, y, z, vx, vy, vz) from LOCAL_POSITION_NED,
        or None if no message has arrived yet.  Drains the socket so the value
        is always fresh.
        """
        msg = None
        while True:
            m = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if m is None:
                break
            msg = m
        if msg is None:
            return None
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz

    def local_ned_blocking(self, timeout: float = 2.0):
        msg = self.mav.recv_match(type='LOCAL_POSITION_NED',
                                   blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError("No LOCAL_POSITION_NED received")
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz

    # ------------------------------------------------------------------
    # Approach A — discrete position targets with settling detection
    # ------------------------------------------------------------------

    def go_to(self, x: float, y: float, z: float,
               tol_pos: float = 0.05,
               tol_vel: float = 0.05,
               timeout: float = 20.0,
               rate_hz: float = 10.0) -> bool:
        """
        Command position (x, y, z) NED and block until settled.

        Keeps resending the target at rate_hz so it reaches ArduPilot even
        if a packet is dropped.  For SubMode::Pos there is no GUID_TIMEOUT
        (position hold is indefinite), but resending is still good practice.

        tol_pos : acceptance radius in metres
        tol_vel : speed threshold in m/s — avoids declaring "arrived" while
                  still coasting through the target
        Returns True on success, False on timeout.
        """
        print(f"  go_to ({x:.3f}, {y:.3f}, {z:.3f})")
        deadline = time.monotonic() + timeout
        interval = 1.0 / rate_hz

        while time.monotonic() < deadline:
            self._send_pos_target(x, y, z, mask=MASK_POS)

            p = self.local_ned()
            if p is not None:
                pos_err = math.sqrt((p[0]-x)**2 + (p[1]-y)**2 + (p[2]-z)**2)
                speed   = math.sqrt(p[3]**2 + p[4]**2 + p[5]**2)
                if pos_err < tol_pos and speed < tol_vel:
                    print(f"    settled  err={pos_err*100:.1f} cm  "
                          f"spd={speed*100:.1f} cm/s")
                    return True

            time.sleep(interval)

        print(f"    timeout after {timeout:.0f} s")
        return False

    # ------------------------------------------------------------------
    # Approach B — PVA trajectory streaming
    # ------------------------------------------------------------------

    def stream_pva(self, trajectory: list, rate_hz: float = 50.0) -> None:
        """
        Stream a precomputed PVA trajectory at rate_hz.

        trajectory : list of (x, y, z, vx, vy, vz, ax, ay, az) tuples,
                     sampled at 1/rate_hz intervals.

        After the trajectory finishes, the final position is held as a
        position-only target (SubMode::Pos, no timeout).

        Note: time.sleep() is not a hard real-time guarantee.  For a PoC
        on a lightly loaded system this is sufficient; for production use
        a dedicated timer thread or asyncio loop is preferred.
        """
        print(f"  stream_pva  {len(trajectory)} pts @ {rate_hz:.0f} Hz")
        dt = 1.0 / rate_hz

        for pt in trajectory:
            t0 = time.monotonic()
            self._send_pos_target(*pt, mask=MASK_PVA)
            elapsed = time.monotonic() - t0
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

        # Hold the final position
        final = trajectory[-1]
        print(f"  holding ({final[0]:.3f}, {final[1]:.3f}, {final[2]:.3f})")
        self._send_pos_target(final[0], final[1], final[2], mask=MASK_POS)


# ---------------------------------------------------------------------------
# Trajectory generators
# ---------------------------------------------------------------------------

def trapezoid_segment(p0, p1, v_max: float, a_max: float,
                       dt: float = 0.02) -> list:
    """
    Minimum-time trapezoidal velocity profile from p0 to p1 in 3D NED.

    Produces a triangle profile (never reaching v_max) when the segment is
    short.  Returns a list of (x,y,z, vx,vy,vz, ax,ay,az) tuples at dt
    intervals suitable for stream_pva().
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    disp = p1 - p0
    dist = float(np.linalg.norm(disp))

    if dist < 1e-4:
        return []

    unit = disp / dist

    d_ramp = v_max**2 / (2.0 * a_max)

    if 2.0 * d_ramp >= dist:
        # Triangle profile — can't reach v_max
        v_peak  = math.sqrt(a_max * dist)
        t_ramp  = v_peak / a_max
        t_cruise = 0.0
        d_ramp  = dist / 2.0
    else:
        v_peak  = v_max
        t_ramp  = v_max / a_max
        t_cruise = (dist - 2.0 * d_ramp) / v_max

    t_total = 2.0 * t_ramp + t_cruise
    points = []
    t = 0.0

    while t <= t_total + dt * 0.5:
        if t < t_ramp:
            s   = 0.5 * a_max * t**2
            v_s = a_max * t
            a_s = a_max
        elif t < t_ramp + t_cruise:
            s   = d_ramp + v_peak * (t - t_ramp)
            v_s = v_peak
            a_s = 0.0
        else:
            tau = t - t_ramp - t_cruise
            s   = d_ramp + v_peak * t_cruise + v_peak * tau - 0.5 * a_max * tau**2
            v_s = v_peak - a_max * tau
            a_s = -a_max

        s   = min(max(s, 0.0), dist)
        v_s = max(v_s, 0.0)

        pos = p0 + s * unit
        vel = v_s * unit
        acc = a_s * unit
        points.append((*pos.tolist(), *vel.tolist(), *acc.tolist()))
        t += dt

    return points


def gen_square(origin, side: float, alt: float,
               v_max: float = 0.3, a_max: float = 0.5,
               dt: float = 0.02) -> list:
    """
    Square trajectory in the NED horizontal plane.
    origin : (x, y) NED metres — start and end point
    side   : side length in metres
    alt    : NED z (negative for altitude above origin, e.g. -1.0 = 1 m AGL)
    Returns concatenated trapezoidal segments through 4 corners and back.
    """
    ox, oy = origin
    corners = [
        (ox,        oy,        alt),
        (ox + side, oy,        alt),
        (ox + side, oy + side, alt),
        (ox,        oy + side, alt),
        (ox,        oy,        alt),
    ]
    traj = []
    for i in range(len(corners) - 1):
        traj.extend(trapezoid_segment(corners[i], corners[i+1],
                                      v_max, a_max, dt))
    return traj


def gen_circle(center, radius: float, alt: float,
               period_s: float = 12.0, dt: float = 0.02) -> list:
    """
    Constant-speed horizontal circle in NED, with analytic velocity and
    centripetal acceleration.

    center   : (x, y) NED metres — circle centre
    radius   : metres
    alt      : NED z (negative for altitude, e.g. -1.0 = 1 m AGL)
    period_s : time for one full revolution
    Trajectory starts at (cx + radius, cy, alt).
    """
    cx, cy = center
    omega  = 2.0 * math.pi / period_s
    traj   = []
    t      = 0.0

    while t < period_s:
        c = math.cos(omega * t)
        s = math.sin(omega * t)
        traj.append((
            cx + radius * c,            # x
            cy + radius * s,            # y
            alt,                        # z
            -radius * omega * s,        # vx
             radius * omega * c,        # vy
            0.0,                        # vz
            -radius * omega**2 * c,     # ax  (centripetal)
            -radius * omega**2 * s,     # ay
            0.0,                        # az
        ))
        t += dt

    return traj


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------

def run_demo(connection: str = 'udpin:0.0.0.0:14550') -> None:
    """
    Assumes the drone is already armed, in GUIDED mode, and hovering.

    Demo A: 0.5 m square using position targets (go_to).
    Demo B: 0.4 m radius circle using PVA streaming (stream_pva).
    """
    ctrl = GuidedController(connection)

    # Read hover position as trajectory origin
    print("Reading hover position…")
    x0, y0, z0 = ctrl.local_ned_blocking()[:3]
    print(f"Hover  x={x0:.3f} y={y0:.3f} z={z0:.3f}  NED (m)")

    # -----------------------------------------------------------------------
    # Demo A: square waypoints, position targets + settling
    # -----------------------------------------------------------------------
    print("\n=== Demo A: 0.5 m square (position targets + settling) ===")
    side = 0.5   # metres

    waypoints = [
        (x0 + side, y0,        z0),
        (x0 + side, y0 + side, z0),
        (x0,        y0 + side, z0),
        (x0,        y0,        z0),   # return to origin
    ]
    for wp in waypoints:
        ctrl.go_to(*wp, tol_pos=0.05, tol_vel=0.05, timeout=20.0)

    time.sleep(2.0)

    # -----------------------------------------------------------------------
    # Demo B: smooth circle, PVA streaming
    # -----------------------------------------------------------------------
    print("\n=== Demo B: 0.4 m radius circle (PVA streaming) ===")
    radius = 0.4
    period = 15.0  # seconds per revolution — keep speed manageable

    # The circle starts at (x0+radius, y0).  Go there first as a waypoint
    # so the transition to streaming is smooth.
    print("  Moving to circle start…")
    ctrl.go_to(x0 + radius, y0, z0, tol_pos=0.05, tol_vel=0.05)

    circle = gen_circle(center=(x0, y0), radius=radius, alt=z0,
                        period_s=period, dt=0.02)
    ctrl.stream_pva(circle, rate_hz=50.0)

    time.sleep(2.0)

    # Return to hover origin
    print("\nReturning to origin…")
    ctrl.go_to(x0, y0, z0)

    ctrl.close()
    print("Done.")


if __name__ == '__main__':
    import sys
    conn = sys.argv[1] if len(sys.argv) > 1 else 'udpin:0.0.0.0:14550'
    run_demo(conn)
