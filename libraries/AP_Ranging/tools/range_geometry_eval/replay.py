#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline four-anchor geometry diagnostics; never supplies navigation data."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import matplotlib.pyplot as plt
import numpy as np

from geometry import leave_one_out, pairwise_violation, plane
from log_reader import read_log


def causal_rows(events, ids, step_ms=200):
    """Use delivered, positive, monotonically acquired samples from each session."""
    latest, sessions = {}, {}
    index = 0
    for now in range(events[0]['arrival'], events[-1]['arrival'] + 1, step_ms):
        while index < len(events) and events[index]['arrival'] <= now:
            event = events[index]
            index += 1
            anchor, acquired, distance = event['id'], event['time'], event['range']
            if anchor not in ids:
                continue
            if anchor in sessions and sessions[anchor] != event['session']:
                latest.pop(anchor, None)
            sessions[anchor] = event['session']
            if (math.isfinite(distance) and 0 < distance < 1000 and acquired <= event['arrival'] and
                    (anchor not in latest or acquired > latest[anchor]['time'])):
                latest[anchor] = event
        fresh = [latest[i] for i in ids if i in latest and 0 <= now - latest[i]['time'] < 300]
        spread = max((e['time'] for e in fresh), default=0) - min((e['time'] for e in fresh), default=0)
        ready = len(fresh) == len(ids) and spread <= 150
        yield {'time_s': (now - events[0]['arrival']) / 1000, 'ready': ready,
               'spread_ms': spread, 'ranges': [e['range'] for e in fresh] if ready else None}


def solve(rows, anchors, noise, executable):
    ready = [r for r in rows if r['ready']]
    lines = []
    for row in ready:
        fields = [4, math.hypot(noise, row['spread_ms'] * .003)]
        for anchor, distance in zip(anchors, row['ranges']):
            fields.extend([*anchor, distance])
        lines.append(' '.join(map(str, fields)))
    if not lines:
        return
    output = subprocess.run([str(executable)], input='\n'.join(lines) + '\n', text=True,
                            capture_output=True, check=True).stdout.splitlines()
    if len(output) != len(ready):
        raise ValueError('solver output count mismatch')
    center, normal, _ = plane(anchors)
    for row, answer in zip(ready, output):
        ok, north, east, down, accuracy = map(float, answer.split())
        row['solver_ok'] = bool(ok)
        row['pairwise_violation_m'] = float(pairwise_violation(anchors, row['ranges']))
        row['hypotheses'] = leave_one_out(anchors, row['ranges'])
        # Positive omitted-range excess is only a hypothesis; 0.10 m is an illustrative threshold.
        row['positive_hypothesis_count'] = sum(any(p['held_out_m'] > .1 for p in candidates)
                                               for candidates in row['hypotheses'])
        row['positive_below_plane_count'] = sum(
            any(p['held_out_m'] > .1 and np.dot(np.array(p['position']) - center, normal) > 0
                for p in candidates) for candidates in row['hypotheses'])
        if ok:
            position = np.array([north, east, down])
            residual = np.linalg.norm(anchors - position, axis=1) - row['ranges']
            row.update(position=position.tolist(), accuracy_m=accuracy,
                       signed_plane_distance_m=float(np.dot(position - center, normal)),
                       rms_m=float(np.sqrt(np.mean(residual**2))), residual_m=residual.tolist())


def summary(rows, start, end):
    chosen = [r for r in rows if start <= r['time_s'] < end]
    ready = [r for r in chosen if r['ready']]
    good = [r for r in ready if r['solver_ok']]

    def fraction(count, total):
        return 100 * count / total if total else None

    return {'window_s': [start, end], 'grid_points': len(chosen), 'ready': len(ready), 'accepted': len(good),
            'ready_pct': fraction(len(ready), len(chosen)),
            'accepted_pct_of_ready': fraction(len(good), len(ready)),
            'above_plane_pct_of_accepted': fraction(sum(r['signed_plane_distance_m'] < 0 for r in good), len(good)),
            'median_position_m': np.median([r['position'] for r in good], axis=0).tolist() if good else None,
            'median_rms_m': float(np.median([r['rms_m'] for r in good])) if good else None,
            'pairwise_violation_pct_of_ready': fraction(sum(r['pairwise_violation_m'] > 1.e-9 for r in ready), len(ready)),
            'multiple_positive_below_plane_pct_of_ready': fraction(
                sum(r['positive_below_plane_count'] >= 2 for r in ready), len(ready)),
            'multiple_positive_hypotheses_pct_of_ready': fraction(
                sum(r['positive_hypothesis_count'] >= 2 for r in ready), len(ready))}


def plot(rows, title, output):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    good = [r for r in rows if r.get('solver_ok')]
    ready = [r for r in rows if r['ready']]
    for axis, key, label in [(axes[0], 'signed_plane_distance_m', 'Signed distance from\nanchor plane (m)'),
                             (axes[1], 'rms_m', 'Four-range fit\nRMS residual (m)')]:
        axis.scatter([r['time_s'] for r in good], [r[key] for r in good], s=3)
        axis.set_ylabel(label)
    axes[0].axhline(0, color='black', lw=.7)
    axes[0].set_title('Positive: below anchor plane; negative: above (NED)', fontsize=9)
    axes[2].scatter([r['time_s'] for r in ready], [r['positive_hypothesis_count'] for r in ready], s=3)
    axes[2].set_ylabel('Possible omitted anchors\nwith >0.10 m positive excess')
    axes[2].set_yticks(range(5))
    axes[3].plot([r['time_s'] for r in rows], [int(r['ready']) for r in rows], label='Four fresh ranges', lw=.8)
    axes[3].scatter([r['time_s'] for r in rows], [int(r.get('solver_ok', False)) - .06 for r in rows],
                    label='Production solver accepts (offset)', s=3)
    axes[3].set_ylabel('Availability / acceptance')
    axes[3].legend(fontsize=8)
    axes[3].set_xlabel('Seconds since first logged RTWR delivery; phase timing is approximate')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle(title + '\nOffline replay, no EKF input; no faulty anchor selected', fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[4])
    args = parser.parse_args()
    repo = args.repo.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).with_name('solver_cli.cpp')
    header = repo / 'libraries/AP_Beacon/AP_Beacon_Ranging_Solver.h'
    manifest = {'grid_ms': 200, 'age_limit_ms_exclusive': 300, 'max_spread_ms': 150, 'sources': {}, 'logs': []}
    files = [source, header, *Path(__file__).parent.glob('*.py')]
    for path in files:
        manifest['sources'][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='range-geometry-') as tmp:
        exe = Path(tmp) / 'solver'
        command = [os.environ.get('CXX', 'clang++'), '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=address,undefined', '-I' + str(header.parent), str(source), '-o', str(exe)]
        subprocess.run(command, check=True)
        manifest['compiler_command'] = command
        fixture = {'ready': True, 'spread_ms': 0}
        fixture['ranges'] = [math.sqrt(3), math.sqrt(11), math.sqrt(11), math.sqrt(11)]
        solve([fixture], np.array([[0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]]), .1, exe)
        if not fixture['solver_ok'] or not np.allclose(fixture['position'], [1, 1, 1], atol=1.e-5):
            raise ValueError('production solver smoke fixture failed')
        for path in args.logs:
            log = read_log(path)
            ids = sorted(log['anchors'])
            if len(ids) != 4:
                raise ValueError('this experiment requires exactly four surveyed anchors')
            anchors = np.array([log['anchors'][i] for i in ids])
            rows = list(causal_rows(log['events'], ids))
            solve(rows, anchors, log['params'].get('RNG_A_NSE', .3), exe)
            center, normal, singular = plane(anchors)
            result = {k: log[k] for k in ['name', 'path', 'sha256', 'anchors', 'params', 'logged_drop_delta']}
            result.update(ids=ids, plane_center=center.tolist(), plane_normal=normal.tolist(),
                          anchor_singular_values=singular.tolist(), rows=rows)
            result['windows'] = {f'{start}-{end}': summary(rows, start, end) for start, end in
                                 [(20, 110), (130, 150), (160, 200), (220, 235), (250, 270), (270, 300), (250, 300)]}
            (args.output / (log['name'] + '.json')).write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
            plot(rows, log['name'], args.output / (log['name'] + '.png'))
            manifest['logs'].append({k: result[k] for k in ['name', 'path', 'sha256', 'windows']})
            print(log['name'], json.dumps(result['windows']))
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
