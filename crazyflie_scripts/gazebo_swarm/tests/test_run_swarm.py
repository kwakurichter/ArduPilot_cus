#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Tests of run_swarm.py file generation and swarm_clock.py (no Gazebo or SITL needed).

    python3 -m unittest discover -s crazyflie_scripts/gazebo_swarm/tests
"""

import json
import re
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import run_swarm  # noqa: E402
from swarm_clock import ClockSync  # noqa: E402

WRAPPER_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="crazyflie_2_1_brushless_with_ardupilot">
    <include><uri>model://crazyflie_2_1_brushless</uri></include>
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>crazyflie_2_1_brushless::m1_prop</link_name>
    </plugin>
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>crazyflie_2_1_brushless::imu_link::imu_sensor</imuName>
      <control channel="0"><jointName>crazyflie_2_1_brushless::m1_joint</jointName></control>
    </plugin>
  </model>
</sdf>
"""

BODY_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="crazyflie_2_1_brushless">
    <link name="base_link"><inertial><mass>0.035679</mass></inertial></link>
  </model>
</sdf>
"""

BODY_CONFIG = """<?xml version="1.0"?>
<model><name>crazyflie_2_1_brushless</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>
"""

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="crazyflie_2_1_brushless_runway">
    <physics name="2ms" type="ignore"><max_step_size>0.002</max_step_size></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <scene><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows></light>
    <include>
      <uri>model://crazyflie_2_1_brushless_with_ardupilot</uri>
      <name>crazyflie</name>
    </include>
  </world>
</sdf>
"""


def make_model_repo(root):
    gazebo = root / "Gazebo"
    (gazebo / "models" / run_swarm.WRAPPER_MODEL).mkdir(parents=True)
    (gazebo / "models" / run_swarm.BODY_MODEL / "meshes").mkdir(parents=True)
    (gazebo / "worlds").mkdir()
    (gazebo / "models" / run_swarm.WRAPPER_MODEL / "model.sdf").write_text(WRAPPER_SDF)
    (gazebo / "models" / run_swarm.BODY_MODEL / "model.sdf").write_text(BODY_SDF)
    (gazebo / "models" / run_swarm.BODY_MODEL / "model.config").write_text(BODY_CONFIG)
    (gazebo / "models" / run_swarm.BODY_MODEL / "meshes" / "body.dae").write_text("mesh")
    (gazebo / "worlds" / run_swarm.SOURCE_WORLD).write_text(WORLD_SDF)
    return root


class Geometry(unittest.TestCase):

    def test_pad_frame_mapping(self):
        # Gazebo +x is east and +y is north for this model
        self.assertEqual(run_swarm.pad_north_east(1.0, 2.0), (2.0, 1.0))

    def test_grid_is_centred_and_spaced(self):
        positions = run_swarm.layout(4, "grid", 0.6, 0)
        self.assertAlmostEqual(sum(x for x, _ in positions), 0.0)
        self.assertAlmostEqual(sum(y for _, y in positions), 0.0)
        self.assertAlmostEqual(run_swarm.min_separation(positions), 0.6)

    def test_circle(self):
        positions = run_swarm.layout(4, "circle", 0, 1.2)
        self.assertAlmostEqual(positions[0][0], 0.0)
        self.assertAlmostEqual(positions[0][1], 1.2)   # vehicle 1 north of centre
        for x, y in positions:
            self.assertAlmostEqual(x * x + y * y, 1.44)

    def test_lanes(self):
        self.assertEqual(run_swarm.lane_altitudes(3, 1.0, 4, 0.25), [0.75, 1.0, 1.25])
        self.assertEqual(run_swarm.lane_altitudes(6, 1.0, 4, 0.25)[4], 0.625)

    def test_offset_location(self):
        lat, lon = run_swarm.offset_location(-35.0, 149.0, 100.0, 0.0)
        self.assertAlmostEqual((lat + 35.0) * 111319.49, 100.0, delta=0.1)
        self.assertEqual(lon, 149.0)


class Generation(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        make_model_repo(self.root / "models_repo")
        self.binary = self.root / "arducopter"
        self.binary.write_text("")

    def tearDown(self):
        self.tmp.cleanup()

    def prepare(self, *extra):
        argv = ["--models-dir", str(self.root / "models_repo"), "--ardupilot-bin", str(self.binary),
                "--run-dir", str(self.root / "run"), "--dry-run"] + list(extra)
        args = run_swarm.parse_args(argv)
        plan, env = run_swarm.prepare(args, lambda text: None)
        return args, plan, env

    def wrapper(self, index):
        tree = ET.parse(self.root / "run" / "models" / run_swarm.wrapper_alias(index) / "model.sdf")
        return tree.getroot().find("model")

    def test_wrappers_have_own_ports_and_time_sync(self):
        self.prepare("--count", "4")
        names = set()
        for index in range(4):
            model = self.wrapper(index)
            plugin = model.find("./plugin[@name='ArduPilotPlugin']")
            names.add(model.get("name"))
            self.assertEqual(plugin.findtext("fdm_port_in"), str(9002 + 10 * index))
            self.assertEqual(plugin.findtext("no_time_sync"), "0")
            self.assertIsNone(plugin.find("sensor"))
            self.assertEqual(model.findtext("./include/uri"), "model://crazyflie_2_1_brushless")
        self.assertEqual(len(names), 4)

    def test_indoor_adds_tof(self):
        self.prepare("--count", "2", "--profile", "indoor")
        model = self.wrapper(1)
        body = "crazyflie_swarm_body_1"
        self.assertEqual(model.findtext("./include/uri"), f"model://{body}")
        self.assertTrue(model.findtext(".//imuName").startswith(f"{body}::"))
        self.assertTrue(model.findtext(".//link_name").startswith(f"{body}::"))
        self.assertTrue(model.findtext(".//jointName").startswith(f"{body}::"))
        mapping = model.find("./plugin[@name='ArduPilotPlugin']/sensor")
        self.assertEqual(mapping.findtext("type"), "range")
        self.assertEqual(mapping.findtext("index"), "1")
        body_model = ET.parse(self.root / "run" / "models" / body / "model.sdf").getroot().find("model")
        self.assertEqual(body_model.get("name"), body)
        lidar = body_model.find("./link[@name='base_link']/sensor[@type='gpu_lidar']")
        self.assertEqual(lidar.findtext("topic"), mapping.findtext("topic"))
        self.assertTrue((self.root / "run" / "models" / body / "meshes" / "body.dae").is_file())
        world = ET.parse(self.root / "run" / "world.sdf").getroot().find("world")
        self.assertIsNotNone(world.find("./plugin[@filename='gz-sim-sensors-system']"))

    def test_world(self):
        _, plan, _ = self.prepare("--count", "3", "--layout", "circle")
        world = ET.parse(plan["world"]).getroot().find("world")
        uris = [i.findtext("uri") for i in world.findall("include")]
        self.assertEqual(uris, [f"model://crazyflie_swarm_{i}" for i in range(3)])
        self.assertIsNone(world.find("./plugin[@filename='gz-sim-sensors-system']"))
        self.assertEqual(world.findtext("scene/shadows"), "false")
        pose = [float(v) for v in world.findall("include")[0].findtext("pose").split()]
        self.assertAlmostEqual(pose[1], 1.2)   # north of centre

    def test_site_module_matches_layout(self):
        _, plan, _ = self.prepare("--count", "4", "--home=-35.5,149.25,600")
        for v in plan["vehicles"]:
            text = (Path(v["dir"]) / "scripts" / "modules" / "swarm_site.lua").read_text()
            self.assertIn(f"index = {v['index']},", text)
            self.assertIn(f"sysid = {v['sysid']},", text)
            self.assertIn("origin_lat_e7 = -355000000,", text)
            self.assertIn("origin_lng_e7 = 1492500000,", text)
            self.assertIn("origin_alt_cm = 60000,", text)
            self.assertIn(f"pad_n = {v['pad_n']:.4f},", text)
            self.assertIn(f"pad_e = {v['pad_e']:.4f},", text)
            self.assertTrue((Path(v["dir"]) / "scripts" / "swarm_hover.lua").is_file())
            self.assertTrue((Path(v["dir"]) / "scripts" / "modules" / "swarm_common.lua").is_file())
            self.assertIn('mission = "hover",', text)
            self.assertEqual((Path(v["dir"]) / "agent.parm").read_text(), "P2P_SWARM_SIZE 4\n")

    def test_site_module_parses_as_lua(self):
        try:
            import lupa
        except ImportError:
            self.skipTest("lupa is not installed")
        _, plan, _ = self.prepare("--count", "2")
        text = (Path(plan["vehicles"][1]["dir"]) / "scripts" / "modules" / "swarm_site.lua").read_text()
        site = lupa.LuaRuntime().execute(text)
        self.assertEqual(site.sysid, 2)
        self.assertEqual(list(site.sysids.values()), [1, 2])
        self.assertIsInstance(site.origin_lat_e7, int)

    def test_sitl_command_gazebo(self):
        _, plan, _ = self.prepare("--count", "2")
        command = plan["vehicles"][1]["command"]
        self.assertIn("JSON:127.0.0.1", command)
        self.assertEqual(command[command.index("--serial0") + 1], "udpclient:127.0.0.1:5770")
        self.assertEqual(command[command.index("--sysid") + 1], "2")
        self.assertIn("-I1", command)
        defaults = command[command.index("--defaults") + 1].split(",")
        self.assertTrue(defaults[0].endswith("default_params/copter.parm"))
        self.assertTrue(defaults[1].endswith("crazyflie_sitl_gazebo.parm"))
        self.assertTrue(defaults[-1].endswith("agent_1/agent.parm"))
        # every vehicle shares the world origin as home; Gazebo places them
        homes = {v["command"][v["command"].index("--home") + 1] for v in plan["vehicles"]}
        self.assertEqual(len(homes), 1)

    def test_sitl_physics_homes_on_pads(self):
        _, plan, _ = self.prepare("--count", "2", "--physics", "sitl", "--profile", "indoor")
        command = plan["vehicles"][0]["command"]
        self.assertEqual(command[command.index("--model") + 1], "quad")
        defaults = command[command.index("--defaults") + 1]
        self.assertNotIn("crazyflie_sitl_gazebo.parm", defaults)
        self.assertIn("sitl_physics_indoor.parm", defaults)
        homes = [v["command"][v["command"].index("--home") + 1] for v in plan["vehicles"]]
        self.assertNotEqual(homes[0], homes[1])
        self.assertFalse((self.root / "run" / "models").exists())

    def test_launch_manifest(self):
        _, plan, _ = self.prepare("--count", "2")
        manifest = json.loads((self.root / "run" / "launch.json").read_text())
        self.assertEqual(manifest["count"], 2)
        self.assertEqual([v["mavlink_udp"] for v in manifest["vehicles"]], [5760, 5770])
        self.assertEqual(manifest["gazebo_commands"][0][:3], ["gz", "sim", "-v1"])

    def test_refuses_low_lanes_and_crowded_pads(self):
        with self.assertRaises(SystemExit):
            run_swarm.parse_args(["--alt", "0.5", "--count", "4"])
        with self.assertRaises(RuntimeError):
            self.prepare("--count", "4", "--spacing", "0.1")

    def test_parameter_files_exist(self):
        for physics in ("gazebo", "sitl"):
            for profile in ("gps", "indoor"):
                args = run_swarm.parse_args(["--physics", physics, "--profile", profile])
                for path in run_swarm.parameter_files(args):
                    self.assertTrue(path.is_file(), path)
                    for line in path.read_text().splitlines():
                        line = line.split("#")[0].strip()
                        if line:
                            self.assertRegex(line, re.compile(r"^[A-Z0-9_]+\s+-?[0-9.]+$"), path)


class Clock(unittest.TestCase):

    def exchange(self, clock, host_ns, offset_ns, rate, rtt_ns, asym=0.5):
        """One TIMESYNC round trip against a vehicle clock fc = rate * host + offset."""
        ts1 = clock.request(host_ns)
        at_vehicle = host_ns + rtt_ns * asym
        tc1 = int(rate * at_vehicle + offset_ns)
        return clock.reply(tc1, ts1, host_ns + rtt_ns)

    def test_offset_and_drift(self):
        clock = ClockSync(1)
        rate = 1.0 - 120e-6          # simulated clock slower than the host
        offset = -950_000_000_000    # ns; the vehicle booted after the host
        for k in range(60):
            host = 1_000_000_000_000 + k * 200_000_000
            rtt = 300_000 + (k % 7) * 400_000
            self.assertTrue(self.exchange(clock, host, offset, rate, rtt))
        self.assertTrue(clock.ready)
        status = clock.status()
        self.assertAlmostEqual(status["drift_ppm"], -120.0, delta=2.0)
        host_us = 1_000_000_000 + 20_000_000   # 8 s past the last sample
        expected_us = (rate * host_us * 1000 + offset) / 1000
        self.assertAlmostEqual(clock.to_fc_us(host_us), expected_us, delta=100)
        self.assertAlmostEqual(clock.to_host_us(clock.to_fc_us(host_us)), host_us, delta=1)

    def test_frozen_mapping_ignores_later_samples(self):
        clock = ClockSync(1)
        for k in range(20):
            self.exchange(clock, 1_000_000_000_000 + k * 200_000_000, -900_000_000_000, 1.0, 500_000)
        frozen = clock.frozen()
        before = frozen.to_fc_us(1_005_000_000)
        for k in range(20, 80):   # the vehicle clock jumps by 50 ms
            self.exchange(clock, 1_000_000_000_000 + k * 200_000_000, -899_950_000_000, 1.0, 500_000)
        self.assertEqual(frozen.to_fc_us(1_005_000_000), before)
        self.assertGreater(clock.to_fc_us(1_005_000_000) - before, 40_000)
        self.assertIsNone(ClockSync(2).frozen())

    def test_not_ready_and_foreign_replies(self):
        clock = ClockSync(1)
        self.assertIsNone(clock.to_fc_us(0))
        self.assertFalse(clock.reply(123, 456, 789))   # not our request
        for k in range(5):
            self.exchange(clock, k * 200_000_000, 0, 1.0, 100_000)
        self.assertFalse(clock.ready)


if __name__ == "__main__":
    unittest.main()
