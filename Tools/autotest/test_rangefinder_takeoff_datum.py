# AP_FLAKE8_CLEAN
"""SITL regression for a range-primary takeoff through a barometer disturbance.

Starts its own local simulator; does not connect to an aircraft. Use --aiding
flow with a SITL build configured with HAL_CF21=1 and AP_SYSLINK_ENABLED=0 to
exercise the Crazyflie's existing GUIDED pre-arm behavior without GPS.
"""
import argparse
import json
import math
import os
import socket
import subprocess
import time
from collections import defaultdict
import statistics
from pathlib import Path
from pymavlink import mavutil


def validate(root, height_source, loss):
    rows = defaultdict(list)
    files = sorted((root / 'logs').glob('*.BIN'))
    if not files:
        raise RuntimeError('No simulator DataFlash log')
    reader = mavutil.mavlink_connection(str(files[-1]))
    wanted = ['ARM', 'MODE', 'RFND', 'SIM2', 'CTUN', 'MAVC', 'XKF5']
    while True:
        msg = reader.recv_match(type=wanted)
        if msg is None:
            break
        rows[msg.get_type()].append(msg.to_dict())
    arm = next(r['TimeUS'] / 1e6 for r in rows['ARM'] if r['ArmState'] == 1)

    def window(kind, start, end):
        return [r for r in rows[kind] if arm + start <= r['TimeUS'] / 1e6 <= arm + end
                and r.get('C', 0) == 0]

    def mean(kind, field, start=20, end=25):
        samples = window(kind, start, end)
        if not samples:
            raise RuntimeError(f'No {kind} samples in measurement window')
        return statistics.mean(r[field] for r in samples)

    result = {'mean_range_m': mean('RFND', 'Dist'),
              'mean_estimated_altitude_m': mean('CTUN', 'Alt'),
              'mean_terrain_offset_m': mean('XKF5', 'TOfs'),
              'simulated_rise_m': mean('SIM2', 'PD', -1, 0) - mean('SIM2', 'PD')}
    failures = []
    commands = rows['MAVC']
    if not any(r['Cmd'] == 22 and r['Res'] == 0 and r['Z'] == 1 for r in commands):
        failures.append('No accepted 1 m takeoff')
    if not any(r['Cmd'] == 21 and r['Res'] == 0 for r in commands):
        failures.append('No accepted LAND')
    if not any(r['ArmState'] == 0 and r['TimeUS'] / 1e6 > arm for r in rows['ARM']):
        failures.append('No autopilot disarm after landing')
    if any(r['Mode'] != 4 for r in window('MODE', 0, 26)):
        failures.append('Unexpected mode change during hover')
    if height_source == 2:
        for key in ['mean_range_m', 'simulated_rise_m']:
            if abs(result[key] - 1) > .15:
                failures.append(f'{key} exceeds 15 cm error: {result[key]:.3f}')
    if loss != 'none':
        # The fault starts relative to the armed heartbeat; allow up to 1 s
        # before sampling. State must become invalid, then recover as Good.
        expected = {'nodata': 1, 'low': 2, 'high': 3}[loss]
        if not any(r['Stat'] == expected for r in window('RFND', 13, 15)):
            failures.append('Injected range fault was not reported invalid')
        if not all(r['Stat'] == 4 for r in window('RFND', 20, 25)):
            failures.append('Rangefinder failed to recover')
    result['failures'] = failures
    (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if failures:
        raise RuntimeError('; '.join(failures))
    print('PASS: rangefinder takeoff datum')


def main():
    os.environ["MAVLINK20"] = "1"
    mavutil.set_dialect("ardupilotmega")
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('--binary', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--aiding', choices=['gps', 'flow'], default='gps')
    p.add_argument('--height-source', type=int, choices=[1, 2, 3], default=2)
    p.add_argument('--loss', choices=['none', 'nodata', 'low', 'high'], default='none')
    p.add_argument('--minimum', type=float, default=.2)
    p.add_argument('--dip', type=float, default=-3)
    p.add_argument('--clearance', type=float, default=.1)
    p.add_argument('--port', type=int, default=26110)
    a = p.parse_args()
    repo = Path(__file__).resolve().parents[2]
    root = a.output.resolve() / a.name
    root.mkdir(parents=True, exist_ok=False)
    params = {'FS_THR_ENABLE': 0, 'LOG_DISARMED': 1, 'SIM_SPEEDUP': 4,
              'RNGFND1_TYPE': 10 if a.loss == 'nodata' else 100, 'RNGFND1_MIN': a.minimum, 'RNGFND1_MAX': 3,
              'RNGFND1_GNDCLR': a.clearance, 'RNGFND1_ORIENT': 25,
              'EK3_SRC1_POSZ': a.height_source, 'EK3_SRC1_VELZ': 0, 'EK3_SRC1_POSXY': 3,
              'EK3_SRC1_VELXY': 3, 'EK3_SRC1_YAW': 1, 'GPS1_TYPE': 1,
              'FLOW_TYPE': 10, 'SIM_FLOW_ENABLE': 1, 'GND_EFFECT_COMP': 1,
              'EK3_GND_EFF_DZ': 4, 'WP_RFND_USE': 0,
              'EK3_RNG_M_NSE': .2, 'EK3_ALT_M_NSE': 2}
    if a.aiding == 'flow':
        params.update(EK3_SRC1_POSXY=0, EK3_SRC1_VELXY=5, EK3_SRC1_YAW=0, GPS1_TYPE=0)
    (root / 'test.parm').write_text(''.join(f'{k} {v}\n' for k, v in params.items()))
    console = (root / 'console.log').open('w')
    log = (root / 'host.jsonl').open('w')
    proc = subprocess.Popen([str(a.binary or repo / 'build/sitl/bin/arducopter'), '--model', 'quad',
                             '--home', '47.397742,8.545594,0,0', '-I', str(a.port // 100 - 200),
                             '--base-port', str(a.port), '--rc-in-port', str(a.port + 3),
                             '--serial0', f'tcp:{a.port}:wait', '--defaults',
                             f'{repo}/Tools/autotest/default_params/copter.parm,{root}/test.parm', '--wipe'],
                            cwd=root, stdout=console, stderr=subprocess.STDOUT)
    link = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                link = mavutil.mavlink_connection(f'tcp:127.0.0.1:{a.port}', source_system=255, source_component=191)
                break
            except (ConnectionRefusedError, socket.error):
                time.sleep(.1)
        if link is None:
            raise RuntimeError('Local simulator did not start')
        cache = {}
        boot = 0
        heartbeat_at = 0
        sim_ground_alt = None
        armed = None

        def record(event, **kw):
            log.write(json.dumps(dict(event=event, boot=boot, **kw), default=str) + '\n')
            log.flush()

        def pump():
            nonlocal boot, heartbeat_at, sim_ground_alt
            msg = link.recv_match(blocking=True, timeout=.02)
            if msg:
                typ = msg.get_type()
                cache[typ] = msg
                if hasattr(msg, 'time_boot_ms'):
                    boot = max(boot, msg.time_boot_ms / 1000)
                record('message', message=msg.to_dict())
                if typ == 'SIM_STATE' and a.loss == 'nodata':
                    if sim_ground_alt is None:
                        sim_ground_alt = msg.alt
                    # Feed simulated truth, never the altitude estimate being tested.
                    drop = armed is not None and 12 <= boot - armed < 15
                    if not drop:
                        tilt = max(.7, math.cos(msg.roll) * math.cos(msg.pitch))
                        distance_cm = max(0, min(65535, round(100 * (msg.alt - sim_ground_alt) / tilt)))
                        link.mav.distance_sensor_send(int(boot * 1000), 0, 300, distance_cm, 0, 0, 25, 0)

                if typ == 'STATUSTEXT':
                    print(a.name, boot, msg.text, flush=True)
            if time.monotonic() - heartbeat_at > .2:
                link.mav.heartbeat_send(6, 8, 0, 0, 4)
                heartbeat_at = time.monotonic()

        def wait_for(predicate, timeout=15):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                pump()
                if predicate():
                    return
            raise RuntimeError(f'Wait timed out at boot {boot}')

        def cmd(command, *values):
            record('command', command=command, params=values)
            link.mav.command_long_send(1, 1, command, 0, *(list(values) + [0] * (7 - len(values))))
        wait_for(lambda: 'HEARTBEAT' in cache)
        for msgid, hz in [(32, 20), (30, 10), (193, 5), (242, 2), (49, 2), (132, 10), (108, 20)]:
            cmd(511, msgid, 1e6 / hz)
        wait_for(lambda: boot > 30)
        if a.aiding == 'flow':
            link.mav.command_int_send(1, 1, 0, 611, 0, 0, 0, 0, 0, 0, 473977420, 85455940, 0)
        cmd(512, 49)
        wait_for(lambda: 'GPS_GLOBAL_ORIGIN' in cache and 'LOCAL_POSITION_NED' in cache)
        origin = cache['GPS_GLOBAL_ORIGIN']
        ground = cache['LOCAL_POSITION_NED']
        home_alt = origin.altitude / 1000 - ground.z
        link.mav.command_int_send(1, 1, 0, 179, 0, 0, 0, 0, 0, 0, 473977420, 85455940, home_alt)
        wait_for(lambda: 'HOME_POSITION' in cache)
        home = cache['HOME_POSITION']
        target_z = (origin.altitude - home.altitude) / 1000 - 1
        record('reference', home=home.to_dict(), ground=ground.to_dict(), target_z=target_z)
        cmd(176, 1, 4)
        wait_for(lambda: cache['HEARTBEAT'].custom_mode == 4)
        cmd(400, 1)
        wait_for(lambda: cache['HEARTBEAT'].base_mode & 128)
        armed = boot
        record('armed')
        # Hold the simulated craft armed on the ground during the pressure pulse,
        # then restore baro as it climbs, like the recorded spin-up disturbance.
        last_glitch = None
        sent_takeoff = False
        sent_land = False
        last_target = -1
        fault_state = None
        while boot < armed + 55:
            pump()
            elapsed = boot - armed
            dip = a.dip * max(0, min(1, elapsed / 1.5, (4 - elapsed) / 1.5))
            dip = round(dip, 1)
            if dip != last_glitch:
                link.mav.param_set_send(1, 1, b'SIM_BARO_GLITCH', dip, 9)
                record('baro_injection', value=dip)
                last_glitch = dip
            if elapsed >= 2 and not sent_takeoff:
                cmd(22, 0, 0, 0, 0, 0, 0, 1)
                sent_takeoff = True
            if 10 < elapsed < 27 and elapsed - last_target > .2:
                link.mav.set_position_target_local_ned_send(
                    0, 1, 1, 1, 3576, ground.x, ground.y, target_z, 0, 0, 0, 0, 0, 0, 0, 0)
                last_target = elapsed
            if a.loss in ('low', 'high'):
                fault_active = 12 <= elapsed < 15
                if fault_active != fault_state:
                    key, on, off = {'low': ('RNGFND1_MIN', 10, a.minimum),
                                    'high': ('RNGFND1_MAX', .3, 3)}[a.loss]
                    link.mav.param_set_send(1, 1, key.encode(), on if fault_active else off, 9)
                    record('range_fault', active=fault_active, kind=a.loss)
                    fault_state = fault_active
            if elapsed >= 27 and not sent_land:
                cmd(21)
                sent_land = True
            if sent_land and not cache['HEARTBEAT'].base_mode & 128:
                record('disarmed')
                break
        else:
            raise RuntimeError('Simulator did not disarm after LAND')
        record('finished')
        print('Flight sequence complete:', a.name, flush=True)
    finally:
        if link:
            link.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()
        console.close()

    validate(root, a.height_source, a.loss)


if __name__ == "__main__":
    main()
