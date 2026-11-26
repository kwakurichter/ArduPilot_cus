#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Launch a Crazyflie swarm in ArduCopter SITL + Gazebo and start it in sync.

One Gazebo Harmonic world holds every Crazyflie 2.1 Brushless; each vehicle is one
ArduCopter SITL process (JSON backend, lock-step). The vehicles see each other only
through AP_SwarmMesh (SITL multicast) and fly a mission script onboard:

    --mission hover    lua/swarm_hover.lua: every vehicle holds its pad position

The script is the orchestrator: it never arms a vehicle, changes its mode or sends it
a target. It only

1. measures every vehicle's boot clock against the host clock with MAVLink TIMESYNC
   (stock ArduPilot answers; see swarm_clock.py), and
2. once every vehicle reports READY, picks one host instant a few seconds ahead and
   sets CFSW_START on each vehicle to that instant in the vehicle's own clock.

Each mission script then arms and takes off when its millis() reaches CFSW_START, starts
its mission at CFSW_START + climb and lands at CFSW_START + climb + duration, so the
swarm takes off, starts and lands together.

``--physics sitl`` replaces Gazebo with SITL's built-in multicopter (a generic quad, not
a Crazyflie) to exercise the mesh, the clock sync and the mission script without Gazebo.

Examples (from the ArduPilot root, after ./waf copter and building the plugin):

    crazyflie_scripts/gazebo_swarm/run_swarm.py --count 4
    crazyflie_scripts/gazebo_swarm/run_swarm.py --count 2 --profile indoor --headless
    crazyflie_scripts/gazebo_swarm/run_swarm.py --count 3 --physics sitl
"""

import argparse
import fcntl
import json
import math
import os
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
AP_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from swarm_clock import ClockSync  # noqa: E402

PARAMS_DIR = HERE / "params"
LUA_DIR = HERE / "lua"
MISSIONS = {"hover": "swarm_hover.lua"}
AIRFRAME_PARM = AP_ROOT / "crazyflie_scripts" / "crazyflie_sitl_gazebo.parm"
COPTER_PARM = AP_ROOT / "Tools" / "autotest" / "default_params" / "copter.parm"

WRAPPER_MODEL = "crazyflie_2_1_brushless_with_ardupilot"
BODY_MODEL = "crazyflie_2_1_brushless"
SOURCE_WORLD = "crazyflie_2_1_brushless_runway.sdf"

FDM_BASE_PORT = 9002
MAVLINK_BASE_PORT = 5760
PORT_STRIDE = 10
DEFAULT_HOME = (-35.363262, 149.165237, 584.0)
EARTH_RADIUS_M = 6378137.0
SPAWN_Z_M = 0.007
MIN_LANE_ALT_M = 0.30
LOCK_PATH = Path(tempfile.gettempdir()) / "crazyflie_swarm_sim.lock"

# swarm_common.lua states (SWSTATE)
STATE_NAMES = {0: "SITE", 1: "WAIT", 2: "READY", 3: "TAKEOFF", 4: "FLY", 5: "LAND", 6: "DONE", 7: "ABORT"}
S_READY, S_FLY, S_DONE, S_ABORT = 2, 4, 6, 7

TIMESYNC_PERIOD_S = 0.2
HEARTBEAT_PERIOD_S = 1.0
STATUS_PERIOD_S = 2.0
FLOAT_EXACT_MS = 2 ** 24  # CFSW_START is a float parameter


# ---------------------------------------------------------------------- geometry

def layout(count, kind, spacing, radius):
    """Gazebo ENU spawn positions (x east, y north), centred on the world origin."""
    if kind == "circle":
        if count == 1:
            return [(0.0, 0.0)]
        positions = []
        for index in range(count):
            angle = 2.0 * math.pi * index / count  # vehicle 0 on the +y (north) axis
            positions.append((radius * math.sin(angle), radius * math.cos(angle)))
        return positions
    columns = math.ceil(math.sqrt(count))
    rows = math.ceil(count / columns)
    positions = []
    for index in range(count):
        row, column = divmod(index, columns)
        positions.append(((column - (columns - 1) / 2.0) * spacing,
                          ((rows - 1) / 2.0 - row) * spacing))
    return positions


def pad_north_east(x_east, y_north):
    """World north/east of a Gazebo position.

    The model's ArduPilotPlugin uses <gazeboXYZToNED>0 0 0 180 0 90</gazeboXYZToNED>,
    so Gazebo +x is east and +y is north.
    """
    return y_north, x_east


def lane_altitudes(count, alt, lanes, spacing):
    lanes = max(1, min(lanes, count))
    return [alt + (index % lanes - (lanes - 1) / 2.0) * spacing for index in range(count)]


def offset_location(lat, lon, north, east):
    """Flat-earth offset, adequate over a room."""
    dlat = math.degrees(north / EARTH_RADIUS_M)
    dlon = math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def min_separation(points):
    best = math.inf
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            best = min(best, math.dist(points[i], points[j]))
    return best


# ---------------------------------------------------------------------- generated files

def site_lua(index, count, home, pads, profile, args):
    """Text of scripts/modules/swarm_site.lua for vehicle ``index``."""
    lat_e7 = int(round(home[0] * 1e7))
    lon_e7 = int(round(home[1] * 1e7))
    alt_cm = int(round(home[2] * 100))
    sysids = ", ".join(str(i + 1) for i in range(count))
    pad_rows = ",\n".join(f"    {{n = {n:.4f}, e = {e:.4f}}}" for n, e in pads)
    pad_n, pad_e = pads[index]
    return (
        "-- Generated by run_swarm.py for one vehicle; do not edit.\n"
        "return {\n"
        f"  index = {index},\n"
        f"  count = {count},\n"
        f"  sysid = {index + 1},\n"
        f"  sysids = {{{sysids}}},\n"
        f"  origin_lat_e7 = {lat_e7},\n"
        f"  origin_lng_e7 = {lon_e7},\n"
        f"  origin_alt_cm = {alt_cm},\n"
        f"  pad_n = {pad_n:.4f},\n"
        f"  pad_e = {pad_e:.4f},\n"
        f"  pads = {{\n{pad_rows}\n  }},\n"
        f"  profile = \"{profile}\",\n"
        f"  mission = \"{args.mission}\",\n"
        f"  alt = {args.alt:.3f},\n"
        f"  lane_count = {args.lanes},\n"
        f"  lane_spacing = {args.lane_spacing:.3f},\n"
        f"  climb_s = {args.climb:.1f},\n"
        f"  duration = {args.duration:.1f},\n"
        "}\n"
    )


def parameter_files(args):
    files = [COPTER_PARM]
    if args.physics == "gazebo":
        files.append(AIRFRAME_PARM)
    files += [PARAMS_DIR / "common.parm",
              PARAMS_DIR / f"profile_{args.profile}.parm",
              PARAMS_DIR / "swarmmesh_sitl.parm"]
    if args.physics == "sitl" and args.profile == "indoor":
        files.append(PARAMS_DIR / "sitl_physics_indoor.parm")
    files += [Path(p).expanduser().resolve() for p in args.extra_param_file]
    return files


def _rename_prefix(root, old, new):
    for element in root.iter():
        if element.text and old in element.text:
            element.text = element.text.replace(old, new)


def write_body_with_tof(models_src, models_out, index):
    """Copy the body model with a downward gpu_lidar for the JSON rangefinder."""
    alias = f"crazyflie_swarm_body_{index}"
    out = models_out / alias
    shutil.copytree(models_src / BODY_MODEL, out, dirs_exist_ok=True)
    tree = ET.parse(out / "model.sdf")
    model = tree.getroot().find("model")
    model.set("name", alias)
    base_link = model.find("./link[@name='base_link']")
    if base_link is None:
        raise RuntimeError(f"{BODY_MODEL}/model.sdf has no base_link")
    sensor = ET.SubElement(base_link, "sensor", {"name": "tof_sensor", "type": "gpu_lidar"})
    # pitched down; 3 cm forward and 8 mm up so the beam clears the collision box
    ET.SubElement(sensor, "pose").text = "0.03 0 0.008 0 1.5707963267948966 0"
    ET.SubElement(sensor, "topic").text = tof_topic(index)
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "30"
    ET.SubElement(sensor, "visualize").text = "false"
    ray = ET.SubElement(sensor, "ray")
    scan = ET.SubElement(ray, "scan")
    for axis in ("horizontal", "vertical"):
        element = ET.SubElement(scan, axis)
        for tag, value in (("samples", "1"), ("resolution", "1"), ("min_angle", "0"), ("max_angle", "0")):
            ET.SubElement(element, tag).text = value
    distance = ET.SubElement(ray, "range")
    for tag, value in (("min", "0.01"), ("max", "4"), ("resolution", "0.001")):
        ET.SubElement(distance, tag).text = value
    noise = ET.SubElement(ray, "noise")
    for tag, value in (("type", "gaussian"), ("mean", "0"), ("stddev", "0.01")):
        ET.SubElement(noise, tag).text = value
    ET.indent(tree, space="  ")
    tree.write(out / "model.sdf", encoding="utf-8", xml_declaration=True)
    config_path = out / "model.config"
    if config_path.is_file():
        config = ET.parse(config_path)
        name = config.getroot().find("name")
        if name is not None:
            name.text = alias
        config.write(config_path, encoding="utf-8", xml_declaration=True)
    return alias


def tof_topic(index):
    return f"/crazyflie_swarm/{index}/tof"


def wrapper_alias(index):
    return f"crazyflie_swarm_{index}"


def write_wrapper(models_src, models_out, index, profile):
    """Per-vehicle copy of the ArduPilot wrapper model: own FDM port, time sync on."""
    alias = wrapper_alias(index)
    out = models_out / alias
    out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(models_src / WRAPPER_MODEL / "model.sdf")
    model = tree.getroot().find("model")
    model.set("name", alias)
    plugin = model.find("./plugin[@name='ArduPilotPlugin']")
    if plugin is None:
        raise RuntimeError(f"{WRAPPER_MODEL}/model.sdf has no ArduPilotPlugin")
    plugin.find("fdm_port_in").text = str(FDM_BASE_PORT + PORT_STRIDE * index)
    # no_time_sync=1 makes SIM_JSON force AHRS_EKF_TYPE=10 (perfect state, no EKF3)
    plugin.find("no_time_sync").text = "0"
    depends = BODY_MODEL
    if profile == "indoor":
        body = write_body_with_tof(models_src, models_out, index)
        model.find("./include/uri").text = f"model://{body}"
        _rename_prefix(model, f"{BODY_MODEL}::", f"{body}::")
        mapping = ET.SubElement(plugin, "sensor")
        ET.SubElement(mapping, "type").text = "range"
        ET.SubElement(mapping, "index").text = "1"
        ET.SubElement(mapping, "topic").text = tof_topic(index)
        depends = body
    ET.indent(tree, space="  ")
    tree.write(out / "model.sdf", encoding="utf-8", xml_declaration=True)

    config = ET.Element("model")
    ET.SubElement(config, "name").text = alias
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", {"version": "1.9"}).text = "model.sdf"
    ET.SubElement(config, "description").text = "Generated per-vehicle Crazyflie wrapper."
    dependency = ET.SubElement(ET.SubElement(config, "depend"), "model")
    ET.SubElement(dependency, "uri").text = f"model://{depends}"
    ET.SubElement(dependency, "version").text = "1.0"
    config_tree = ET.ElementTree(config)
    ET.indent(config_tree, space="  ")
    config_tree.write(out / "model.config", encoding="utf-8", xml_declaration=True)
    return alias


def write_world(source_world, run_dir, positions, profile):
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    world.set("name", "crazyflie_swarm")
    for include in list(world.findall("include")):
        if include.findtext("uri", default="").strip() == f"model://{WRAPPER_MODEL}":
            world.remove(include)
    sensors = [p for p in world.findall("plugin") if p.get("filename") == "gz-sim-sensors-system"]
    if profile == "indoor":
        if not sensors:
            plugin = ET.Element("plugin", {"filename": "gz-sim-sensors-system",
                                           "name": "gz::sim::systems::Sensors"})
            ET.SubElement(plugin, "render_engine").text = "ogre2"
            world.insert(1, plugin)
    else:
        for plugin in sensors:  # nothing renders without the ToF
            world.remove(plugin)
    shadows = world.find("scene/shadows")
    if shadows is not None:
        shadows.text = "false"
    for light in world.findall("light"):
        cast = light.find("cast_shadows")
        if cast is not None:
            cast.text = "false"
    for index, (x, y) in enumerate(positions):
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = f"model://{wrapper_alias(index)}"
        ET.SubElement(include, "name").text = f"crazyflie_{index + 1}"
        ET.SubElement(include, "pose").text = f"{x:.4f} {y:.4f} {SPAWN_Z_M} 0 0 0"
    path = run_dir / "world.sdf"
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


# ---------------------------------------------------------------------- commands

def sitl_command(binary, index, home, defaults, args):
    command = [
        str(binary), "-w",
        "--speedup", f"{args.speedup:g}",
        "--home", f"{home[0]:.8f},{home[1]:.8f},{home[2]:.2f},0",
        "--sysid", str(index + 1),
        "--serial0", f"udpclient:127.0.0.1:{MAVLINK_BASE_PORT + PORT_STRIDE * index}",
        "--defaults", ",".join(str(p) for p in defaults),
        f"-I{index}",
    ]
    if args.physics == "gazebo":
        command[2:2] = ["--model", "JSON:127.0.0.1", "--sim-address=127.0.0.1"]
    else:
        command[2:2] = ["--model", "quad"]
    return command


def gazebo_commands(world_path, args):
    server = ["gz", "sim", f"-v{args.verbosity}", "-s", "-r", str(world_path)]
    commands = [server]
    if not args.headless:
        commands.append(["gz", "sim", f"-v{args.verbosity}", "-g"])
    return commands


def gazebo_environment(run_dir, models_root, plugin_dir):
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"crazyflie_swarm_{os.getpid()}"
    env["IGN_PARTITION"] = env["GZ_PARTITION"]
    env.setdefault("GZ_IP", "127.0.0.1")
    env.setdefault("IGN_IP", env["GZ_IP"])

    def prepend(name, *paths):
        values = [str(p) for p in paths]
        if env.get(name):
            values.append(env[name])
        env[name] = os.pathsep.join(values)

    prepend("GZ_SIM_RESOURCE_PATH", run_dir / "models", models_root / "models", models_root / "worlds")
    prepend("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_dir)
    return env


# ---------------------------------------------------------------------- resolution

def resolve_binary(args):
    if args.ardupilot_bin:
        binary = Path(args.ardupilot_bin).expanduser().resolve()
    else:
        binary = AP_ROOT / "build" / "sitl" / "bin" / "arducopter"
    if not binary.is_file():
        raise FileNotFoundError(f"ArduCopter SITL not found at {binary}; run "
                                "'./waf configure --board sitl && ./waf copter'")
    return binary


def resolve_models(args):
    value = args.models_dir or os.environ.get("CRAZYFLIE_SITL_MODELS_DIR")
    if not value:
        raise FileNotFoundError("set --models-dir or CRAZYFLIE_SITL_MODELS_DIR to the "
                                "SITL_Models-crazyflie-brushless checkout")
    root = Path(value).expanduser().resolve()
    gazebo = root / "Gazebo" if (root / "Gazebo").is_dir() else root
    for required in (gazebo / "models" / WRAPPER_MODEL / "model.sdf",
                     gazebo / "models" / BODY_MODEL / "model.sdf",
                     gazebo / "worlds" / SOURCE_WORLD):
        if not required.is_file():
            raise FileNotFoundError(f"model file not found: {required}")
    return gazebo


def resolve_plugin_dir(args):
    candidates = [args.plugin_dir, os.environ.get("ARDUPILOT_GAZEBO_PLUGIN_DIR"),
                  AP_ROOT / "build" / "gz_plugin"]
    if os.environ.get("ARDUPILOT_GAZEBO_DIR"):
        candidates.append(Path(os.environ["ARDUPILOT_GAZEBO_DIR"]) / "build")
    for candidate in candidates:
        if candidate and any(Path(candidate).expanduser().glob("libArduPilotPlugin.*")):
            return Path(candidate).expanduser().resolve()
    raise FileNotFoundError("libArduPilotPlugin not found; build crazyflie_scripts/gazebo_swarm/plugin "
                            "into build/gz_plugin or pass --plugin-dir")


def ports_in_use(count):
    busy = []
    for index in range(count):
        base = PORT_STRIDE * index
        checks = [(socket.SOCK_STREAM, MAVLINK_BASE_PORT + base + 2), (socket.SOCK_STREAM, MAVLINK_BASE_PORT + base + 3),
                  (socket.SOCK_DGRAM, FDM_BASE_PORT + base)]
        for kind, port in checks:
            probe = socket.socket(socket.AF_INET, kind)
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                busy.append(port)
            finally:
                probe.close()
    return busy


# ---------------------------------------------------------------------- orchestration

class Vehicle:
    """One SITL vehicle seen from the orchestrator over its serial0 UDP link."""

    def __init__(self, index, mavutil):
        self.index = index
        self.sysid = index + 1
        self.port = MAVLINK_BASE_PORT + PORT_STRIDE * index
        self.conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{self.port}", source_system=255,
                                               source_component=190, dialect="ardupilotmega")
        self.clock = ClockSync(self.sysid)
        self.heartbeat = None
        self.params = {}
        self.state = None
        self.peers = None
        self.tko_ms = None
        self.start_ms = None
        self.start_confirmed = False
        self.start_clock = None   # the clock mapping the start was scheduled with
        self.relative_alt = None
        self.max_alt = 0.0
        self.errors = []      # SWERR while flying the mission, m
        self.armed = False
        self.intervals_requested = False
        self.next_timesync = 0.0
        self.texts = []

    def socket(self):
        return self.conn.port

    def drain(self, now_ns, log):
        while True:
            message = self.conn.recv_msg()
            if message is None:
                return
            if message.get_srcSystem() != self.sysid:
                continue
            kind = message.get_type()
            if kind == "HEARTBEAT" and message.get_srcComponent() == 1:
                self.heartbeat = message
                self.armed = bool(message.base_mode & 128)
            elif kind == "TIMESYNC" and message.tc1 != 0:
                self.clock.reply(message.tc1, message.ts1, now_ns)
            elif kind == "PARAM_VALUE":
                name = message.param_id.rstrip("\x00") if isinstance(message.param_id, str) else \
                    message.param_id.decode().rstrip("\x00")
                self.params[name] = message.param_value
            elif kind == "NAMED_VALUE_FLOAT":
                name = message.name.rstrip("\x00") if isinstance(message.name, str) else message.name.decode()
                if name == "SWSTATE":
                    self.state = int(round(message.value))
                elif name == "SWPEERS":
                    self.peers = int(round(message.value))
                elif name == "SWTKO" and self.tko_ms is None:
                    self.tko_ms = int(round(message.value))
                elif name == "SWERR" and self.state == S_FLY:
                    self.errors.append(message.value)
            elif kind == "GLOBAL_POSITION_INT":
                self.relative_alt = message.relative_alt / 1000.0
                if self.armed:
                    self.max_alt = max(self.max_alt, self.relative_alt)
            elif kind == "STATUSTEXT":
                text = message.text.rstrip("\x00") if isinstance(message.text, str) else message.text.decode()
                self.texts.append(text)
                log(f"[{self.sysid}] {text}")

    def connected(self):
        """True once the vehicle has sent us something we can answer."""
        return bool(getattr(self.conn, "clients", None)) or getattr(self.conn, "last_address", None) is not None

    def send_heartbeat(self, mavlink):
        if self.connected():
            self.conn.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_timesync(self, now_ns):
        if self.connected():
            self.conn.mav.timesync_send(0, self.clock.request(now_ns))

    def request_param(self, name):
        self.conn.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)

    def set_param(self, name, value, mavlink):
        self.conn.mav.param_set_send(self.sysid, 1, name.encode(), float(value), mavlink.MAV_PARAM_TYPE_REAL32)

    def request_intervals(self, mavlink):
        for message_id, hz in ((mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 4.0),
                               (mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1.0)):
            self.conn.mav.command_long_send(self.sysid, 1, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                            message_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self.intervals_requested = True

    def state_name(self):
        return STATE_NAMES.get(self.state, "-") if self.state is not None else "-"


class Orchestrator:
    def __init__(self, vehicles, processes, args, log):
        self.vehicles = vehicles
        self.processes = processes
        self.args = args
        self.log = log
        from pymavlink.dialects.v20 import ardupilotmega
        self.mavlink = ardupilotmega
        self.t0 = time.monotonic()
        self.next_heartbeat = 0.0
        self.next_status = 0.0
        self.phase = "boot"
        self.start_host_us = None

    # -------------------------------------------------------------- loop plumbing

    def poll(self, timeout=0.02):
        select.select([v.socket() for v in self.vehicles], [], [], timeout)
        now_ns = time.monotonic_ns()
        for v in self.vehicles:
            # every link, not only the readable ones: one datagram can carry several
            # messages, and the parser keeps the rest buffered
            v.drain(now_ns, self.log)
        now = time.monotonic()
        if now >= self.next_heartbeat:
            self.next_heartbeat = now + HEARTBEAT_PERIOD_S
            for v in self.vehicles:
                v.send_heartbeat(self.mavlink)
        for v in self.vehicles:
            if v.heartbeat is not None and now >= v.next_timesync:
                v.next_timesync = now + TIMESYNC_PERIOD_S
                v.send_timesync(time.monotonic_ns())
        if now >= self.next_status:
            self.next_status = now + STATUS_PERIOD_S
            self.print_status()
        self.check_processes()

    def check_processes(self):
        for name, process in self.processes:
            status = process.poll()
            if status is not None:
                raise RuntimeError(f"{name} exited with status {status}; see its log in the run directory")

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError(f"timed out after {timeout:.0f} s waiting for {what}")
            self.poll()

    def elapsed(self):
        return time.monotonic() - self.t0

    def print_status(self):
        parts = []
        for v in self.vehicles:
            sync = v.clock.status()
            clock = f"±{sync['residual_ms']:.2f}ms" if sync.get("ready") else f"sync {sync['replies']}"
            alt = f"{v.relative_alt:.2f}m" if v.relative_alt is not None else "-"
            peers = v.peers if v.peers is not None else "-"
            error = f" err {v.errors[-1]:.2f}m" if v.state == S_FLY and v.errors else ""
            parts.append(f"{v.sysid}:{v.state_name()} p{peers} {alt}{error} {clock}")
        self.log(f"t={self.elapsed():6.1f}s [{self.phase}] " + " | ".join(parts))

    # -------------------------------------------------------------- phases

    def run(self):
        args = self.args
        self.phase = "boot"
        self.wait_for(lambda: all(v.heartbeat is not None for v in self.vehicles), args.boot_timeout,
                      "a heartbeat from every vehicle")
        self.log("all vehicles up")

        self.phase = "check"
        for v in self.vehicles:
            v.request_intervals(self.mavlink)
        self.check_parameters()

        self.phase = "sync"
        self.wait_for(lambda: all(v.clock.ready and v.state == S_READY for v in self.vehicles),
                      args.ready_timeout, "every vehicle READY with its clock measured")
        self.schedule_start()

        self.phase = "flight"
        flight_s = args.lead + args.climb + args.duration + args.land_timeout
        self.wait_for(lambda: all(v.state in (S_DONE, S_ABORT) for v in self.vehicles), flight_s,
                      "every vehicle to land")
        self.phase = "done"
        self.print_status()
        return self.summary()

    def check_parameters(self):
        expected = {"AHRS_EKF_TYPE": 3, "P2P_TYPE": 10, "CFSW_START": 0}
        deadline = time.monotonic() + self.args.param_timeout
        next_request = 0.0
        while True:
            missing = [(v, name) for v in self.vehicles for name in expected if name not in v.params]
            if not missing:
                break
            if time.monotonic() > deadline:
                names = sorted({f"{v.sysid}:{n}" for v, n in missing})
                raise TimeoutError(f"no answer for parameters {names} (is the mission script running?)")
            if time.monotonic() >= next_request:
                next_request = time.monotonic() + 1.0
                for v, name in missing:
                    v.request_param(name)
            self.poll()
        for v in self.vehicles:
            for name, value in expected.items():
                if name != "CFSW_START" and round(v.params[name]) != value:
                    raise RuntimeError(f"vehicle {v.sysid}: {name} = {v.params[name]:g}, expected {value}")
        self.log("parameters checked: EKF3, SwarmMesh SITL, mission script loaded")

    def schedule_start(self):
        """Set CFSW_START on every vehicle to one host instant, in each vehicle's clock."""
        args = self.args
        for attempt in range(3):
            self.phase = "start"
            host_us = time.monotonic_ns() // 1000 + int(args.lead * 1e6)
            for v in self.vehicles:
                v.start_clock = v.clock.frozen()
                fc_us = v.start_clock.to_fc_us(host_us)
                v.start_ms = int(round(fc_us / 1000.0))
                v.start_confirmed = False
                if v.start_ms >= FLOAT_EXACT_MS:
                    raise RuntimeError(f"vehicle {v.sysid} has run for {v.start_ms / 3.6e6:.1f} h; "
                                       "CFSW_START would lose millisecond precision")
            self.log(f"start in {args.lead:.1f} s: " + ", ".join(f"{v.sysid}@{v.start_ms}ms" for v in self.vehicles))
            confirm_by = time.monotonic() + max(0.5, args.lead - 1.0)
            next_send = 0.0
            while time.monotonic() < confirm_by:
                pending = [v for v in self.vehicles if not v.start_confirmed]
                if not pending:
                    self.start_host_us = host_us
                    self.phase = "flight"
                    return
                if time.monotonic() >= next_send:
                    next_send = time.monotonic() + 0.3
                    for v in pending:
                        v.params.pop("CFSW_START", None)
                        v.set_param("CFSW_START", v.start_ms, self.mavlink)
                self.poll()
                for v in pending:
                    value = v.params.get("CFSW_START")
                    if value is not None and abs(value - v.start_ms) < 0.5:
                        v.start_confirmed = True
            self.log("start not confirmed by every vehicle in time; clearing it and retrying")
            self.clear_start()
        raise RuntimeError("could not schedule a synchronized start")

    def clear_start(self):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            for v in self.vehicles:
                v.set_param("CFSW_START", 0, self.mavlink)
            self.poll(0.2)

    def summary(self):
        """Per-vehicle results. Takeoff times are mapped to the host clock through the
        mapping each start was scheduled with, so the spread shows how well the vehicles
        acted on one instant (clock fit error + script scheduling), not later fit drift."""
        vehicles = []
        tko_host = []
        for v in self.vehicles:
            sync = v.start_clock.status() if v.start_clock else v.clock.status()
            entry = dict(sysid=v.sysid, final_state=v.state_name(), clock_at_start=sync,
                         start_ms=v.start_ms, takeoff_ms=v.tko_ms, max_alt_m=round(v.max_alt, 3))
            if v.errors:
                # skip the first 2 s of FLY: the climb overshoot settles there
                settled = v.errors[4:] or v.errors
                entry["tracking_error_m"] = dict(mean=round(sum(settled) / len(settled), 3),
                                                 max=round(max(settled), 3), samples=len(settled))
            if v.tko_ms is not None and v.start_clock is not None:
                entry["takeoff_late_ms"] = v.tko_ms - v.start_ms
                host = v.start_clock.to_host_us(v.tko_ms * 1000)
                tko_host.append(host)
                entry["takeoff_host_us"] = host
            vehicles.append(entry)
        result = dict(vehicles=vehicles, all_done=all(v.state == S_DONE for v in self.vehicles))
        if self.start_host_us is not None:
            result["start_host_us"] = self.start_host_us
            residuals = [v.start_clock.status()["residual_ms"] for v in self.vehicles if v.start_clock]
            result["clock_residual_ms_max"] = round(max(residuals), 3)
        if len(tko_host) == len(self.vehicles) and len(tko_host) > 1:
            result["takeoff_spread_ms"] = round((max(tko_host) - min(tko_host)) / 1000.0, 3)
        return result


# ---------------------------------------------------------------------- processes

def stop_processes(processes):
    running = [p for _, p in processes if p.poll() is None]
    for process in running:
        process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 5.0
    for process in running:
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def acquire_lock():
    handle = open(LOCK_PATH, "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(f"another swarm simulation is running ({LOCK_PATH}); the SwarmMesh "
                           "multicast group is shared, so only one swarm can run per host")
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


# ---------------------------------------------------------------------- main

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=4, help="number of vehicles (default 4)")
    parser.add_argument("--allow-large", action="store_true", help="allow more than 16 vehicles (up to 64)")
    parser.add_argument("--layout", choices=("grid", "circle"), default="grid")
    parser.add_argument("--spacing", type=float, default=0.6, help="grid spacing, m (default 0.6)")
    parser.add_argument("--radius", type=float, default=1.2, help="circle radius, m (default 1.2)")
    parser.add_argument("--physics", choices=("gazebo", "sitl"), default="gazebo",
                        help="gazebo: Crazyflie model in Gazebo (default); sitl: SITL's generic quad, no Gazebo")
    parser.add_argument("--profile", choices=("gps", "indoor"), default="gps",
                        help="EKF3 sensor profile (default gps)")
    parser.add_argument("--mission", choices=sorted(MISSIONS), default="hover",
                        help="hover: hold the pads (default)")
    parser.add_argument("--alt", type=float, default=1.0, help="centre altitude, m (default 1.0)")
    parser.add_argument("--lanes", type=int, default=4, help="altitude lanes (default 4)")
    parser.add_argument("--lane-spacing", type=float, default=0.25, help="altitude lane spacing, m (default 0.25)")
    parser.add_argument("--climb", type=float, default=10.0, help="time allowed for the climb, s (default 10)")
    parser.add_argument("--duration", type=float, default=20.0, help="mission time after the climb, s (default 20)")
    parser.add_argument("--lead", type=float, default=5.0, help="start this far after scheduling, s (default 5)")
    parser.add_argument("--home", default=",".join(str(v) for v in DEFAULT_HOME),
                        help="world origin LAT,LON,ALT_M; write --home=LAT,... when LAT is negative (default CMAC)")
    parser.add_argument("--speedup", type=float, default=1.0, help="SITL speedup (sitl physics only)")
    parser.add_argument("--models-dir", help="SITL_Models-crazyflie-brushless checkout "
                                             "(default $CRAZYFLIE_SITL_MODELS_DIR)")
    parser.add_argument("--plugin-dir", help="directory holding libArduPilotPlugin (default build/gz_plugin)")
    parser.add_argument("--ardupilot-bin", help="arducopter SITL binary (default build/sitl/bin/arducopter)")
    parser.add_argument("--extra-param-file", action="append", default=[], help="extra defaults file (repeatable)")
    parser.add_argument("--run-dir", help="output directory (default: a new temporary directory)")
    parser.add_argument("--headless", action="store_true", help="no Gazebo GUI")
    parser.add_argument("--world-only", action="store_true", help="start Gazebo without the vehicles")
    parser.add_argument("--no-monitor", action="store_true",
                        help="do not orchestrate; leave the MAVLink ports to swarm_hub.py or a GCS")
    parser.add_argument("--dry-run", action="store_true", help="generate files and print commands only")
    parser.add_argument("--verbosity", type=int, choices=range(5), default=1, help="Gazebo verbosity")
    parser.add_argument("--boot-timeout", type=float, default=90.0)
    parser.add_argument("--param-timeout", type=float, default=30.0)
    parser.add_argument("--ready-timeout", type=float, default=180.0)
    parser.add_argument("--land-timeout", type=float, default=40.0)
    args = parser.parse_args(argv)

    limit = 64 if args.allow_large else 16
    if not 1 <= args.count <= limit:
        parser.error(f"--count must be between 1 and {limit}")
    if args.physics == "sitl" and args.world_only:
        parser.error("--world-only needs --physics gazebo")
    if args.physics == "gazebo" and args.speedup != 1.0:
        parser.error("--speedup applies to --physics sitl only; Gazebo sets the pace")
    if args.lead < 2.0:
        parser.error("--lead must be at least 2 s")
    try:
        home = tuple(float(v) for v in args.home.split(","))
        assert len(home) == 3
    except (ValueError, AssertionError):
        parser.error("--home must be LAT,LON,ALT_M")
    args.home = home
    lowest = min(lane_altitudes(args.count, args.alt, args.lanes, args.lane_spacing))
    if lowest < MIN_LANE_ALT_M:
        parser.error(f"the lowest altitude lane is {lowest:.2f} m; keep it at or above {MIN_LANE_ALT_M} m")
    return args


def prepare(args, log):
    """Generate the run directory; return (plan dict, Gazebo env or None)."""
    run_dir = Path(args.run_dir).expanduser().resolve() if args.run_dir else \
        Path(tempfile.mkdtemp(prefix="crazyflie_swarm_"))
    if run_dir.exists() and any(run_dir.iterdir()):
        raise RuntimeError(f"run directory {run_dir} is not empty")
    run_dir.mkdir(parents=True, exist_ok=True)

    positions = layout(args.count, args.layout, args.spacing, args.radius)
    separation = min_separation(positions) if args.count > 1 else math.inf
    if separation < 0.15:
        raise RuntimeError(f"vehicles spawn {separation:.2f} m apart; increase --spacing/--radius")
    pads = [pad_north_east(x, y) for x, y in positions]

    binary = None if args.world_only else resolve_binary(args)
    defaults = parameter_files(args)
    for path in defaults:
        if not path.is_file():
            raise FileNotFoundError(f"parameter file not found: {path}")

    env = None
    gz_commands = []
    world_path = None
    if args.physics == "gazebo":
        models_root = resolve_models(args)
        plugin_dir = None if args.dry_run else resolve_plugin_dir(args)
        models_out = run_dir / "models"
        models_out.mkdir()
        for index in range(args.count):
            write_wrapper(models_root / "models", models_out, index, args.profile)
        world_path = write_world(models_root / "worlds" / SOURCE_WORLD, run_dir, positions, args.profile)
        env = gazebo_environment(run_dir, models_root, plugin_dir or Path("<plugin-dir>"))
        gz_commands = gazebo_commands(world_path, args)

    vehicles = []
    for index in range(0 if args.world_only else args.count):
        agent_dir = run_dir / f"agent_{index}"
        (agent_dir / "scripts" / "modules").mkdir(parents=True)
        shutil.copy2(LUA_DIR / MISSIONS[args.mission], agent_dir / "scripts" / MISSIONS[args.mission])
        for module in sorted((LUA_DIR / "modules").glob("*.lua")):
            shutil.copy2(module, agent_dir / "scripts" / "modules" / module.name)
        (agent_dir / "scripts" / "modules" / "swarm_site.lua").write_text(
            site_lua(index, args.count, args.home, pads, args.profile, args))
        agent_parm = agent_dir / "agent.parm"
        agent_parm.write_text(f"P2P_SWARM_SIZE {args.count}\n")
        if args.physics == "gazebo":
            home = args.home  # Gazebo places every vehicle relative to the common origin
        else:
            lat, lon = offset_location(args.home[0], args.home[1], *pads[index])
            home = (lat, lon, args.home[2])
        command = sitl_command(binary, index, home, defaults + [agent_parm], args)
        vehicles.append(dict(index=index, sysid=index + 1, dir=str(agent_dir), command=command,
                             pad_n=pads[index][0], pad_e=pads[index][1],
                             mavlink_udp=MAVLINK_BASE_PORT + PORT_STRIDE * index,
                             json_fdm=FDM_BASE_PORT + PORT_STRIDE * index))

    plan = dict(run_dir=str(run_dir), physics=args.physics, profile=args.profile, count=args.count,
                home=list(args.home), world=str(world_path) if world_path else None,
                gazebo_commands=gz_commands, gazebo_partition=env["GZ_PARTITION"] if env else None,
                parameter_files=[str(p) for p in defaults], vehicles=vehicles,
                mission=dict(name=args.mission, alt=args.alt, lanes=args.lanes, lane_spacing=args.lane_spacing,
                             climb_s=args.climb, duration_s=args.duration, lead_s=args.lead))
    (run_dir / "launch.json").write_text(json.dumps(plan, indent=2) + "\n")
    return plan, env


def main(argv=None):
    args = parse_args(argv)
    log = lambda text: print(text, flush=True)  # noqa: E731
    plan, env = prepare(args, log)
    run_dir = Path(plan["run_dir"])
    log(f"run directory: {run_dir}")
    for v in plan["vehicles"]:
        log(f"vehicle {v['sysid']}: pad N{v['pad_n']:+.2f} E{v['pad_e']:+.2f}, MAVLink UDP {v['mavlink_udp']}"
            + (f", JSON FDM {v['json_fdm']}" if args.physics == "gazebo" else ""))
    if args.dry_run:
        for command in plan["gazebo_commands"]:
            log("gazebo: " + " ".join(command))
        for v in plan["vehicles"]:
            log("sitl:   " + " ".join(v["command"]))
        return 0

    lock = acquire_lock()
    busy = ports_in_use(len(plan["vehicles"]))
    if busy:
        raise RuntimeError(f"ports already in use: {busy}")

    processes = []
    logs = []
    vehicles = []
    try:
        if not args.no_monitor and not args.world_only:
            os.environ.setdefault("MAVLINK20", "1")
            from pymavlink import mavutil
            vehicles = [Vehicle(v["index"], mavutil) for v in plan["vehicles"]]  # bind before SITL starts

        for index, command in enumerate(plan["gazebo_commands"]):
            gz = shutil.which("gz")
            if gz is None:
                raise FileNotFoundError("Gazebo 'gz' not found on PATH (Gazebo Harmonic required)")
            handle = open(run_dir / ("gazebo.log" if index == 0 else "gazebo_gui.log"), "w")
            logs.append(handle)
            processes.append(("gazebo" if index == 0 else "gazebo gui",
                              subprocess.Popen([gz] + command[1:], env=env, stdout=handle, stderr=subprocess.STDOUT)))
            time.sleep(0.75)

        for v in plan["vehicles"]:
            handle = open(Path(v["dir"]) / "sitl.log", "w")
            logs.append(handle)
            processes.append((f"vehicle {v['sysid']} SITL",
                              subprocess.Popen(v["command"], cwd=v["dir"], stdout=handle, stderr=subprocess.STDOUT)))

        if not vehicles:
            log("running; press Ctrl-C to stop")
            while True:
                for name, process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f"{name} exited with status {process.returncode}")
                time.sleep(0.5)

        orchestrator = Orchestrator(vehicles, processes, args, log)
        try:
            summary = orchestrator.run()
        except (RuntimeError, TimeoutError) as error:
            log(f"error: {error}")
            summary = orchestrator.summary()
            summary["error"] = str(error)
        summary["run_dir"] = str(run_dir)
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        log(f"summary: {run_dir / 'summary.json'}")
        if "takeoff_spread_ms" in summary:
            log(f"takeoff spread {summary['takeoff_spread_ms']:.2f} ms (host clock), clock fit residual "
                f"at scheduling up to {summary['clock_residual_ms_max']:.2f} ms")
        for entry in summary["vehicles"]:
            late = entry.get("takeoff_late_ms")
            error = entry.get("tracking_error_m")
            log(f"vehicle {entry['sysid']}: {entry['final_state']}, max alt {entry['max_alt_m']:.2f} m"
                + (f", armed {late} ms after its start time" if late is not None else "")
                + (f", tracking error mean {error['mean']:.2f} m max {error['max']:.2f} m" if error else ""))
        return 0 if summary.get("all_done") and "error" not in summary else 1
    except KeyboardInterrupt:
        log("interrupted")
        return 130
    finally:
        stop_processes(processes)
        for handle in logs:
            handle.close()
        for v in vehicles:
            v.conn.close()
        lock.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
