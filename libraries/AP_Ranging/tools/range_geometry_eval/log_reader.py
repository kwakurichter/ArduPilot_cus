#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Read anchor ranges and survey data for offline geometry diagnostics."""
from collections import Counter
import hashlib

from pymavlink import DFReader


def unwrap_ms(raw_ms, arrival_ms):
    value = int(raw_ms) + (int(arrival_ms) // (1 << 32)) * (1 << 32)
    if value - arrival_ms > (1 << 31):
        value -= 1 << 32
    elif arrival_ms - value > (1 << 31):
        value += 1 << 32
    return value


def read_log(path):
    reader = DFReader.DFReader_binary(str(path))
    events, params, anchors = [], {}, {}
    changes = []
    counts = Counter()
    drops = []
    while True:
        msg = reader.recv_msg()
        if msg is None:
            break
        kind = msg.get_type()
        counts[kind] += 1
        if kind == 'PARM':
            if msg.Name in params and params[msg.Name] != msg.Value and msg.Name.startswith(('RNG_', 'BCN_')):
                changes.append((msg.TimeUS, msg.Name, params[msg.Name], msg.Value))
            params[msg.Name] = msg.Value
        elif kind == 'RTWR':
            arrival = int(msg.TimeUS) // 1000
            events.append({'arrival': arrival, 'time': unwrap_ms(msg.AcqMS, arrival),
                           'id': int(msg.Remote), 'range': float(msg.Dist), 'session': int(msg.Session)})
        elif kind == 'RANC' and msg.En:
            pos = [msg.N, msg.E, msg.D]
            if msg.ID in anchors and pos != anchors[msg.ID]:
                raise ValueError(f'{path}: anchor position changed; segment log first')
            anchors[int(msg.ID)] = pos
        elif kind == 'DSF':
            drops.append(int(msg.Dp))
    if changes:
        raise ValueError(f'{path}: ranging configuration changed; segment log first: {changes}')
    events.sort(key=lambda e: e['arrival'])  # Stable: keep order within a logged batch.
    if not events:
        raise ValueError(f'{path}: no RTWR records')
    if int(params.get('RNG_MODE', -1)) != 1:
        raise ValueError(f'{path}: anchor mode required')
    return {'name': path.stem, 'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'counts': dict(counts), 'params': params, 'anchors': anchors, 'events': events,
            'logged_drop_delta': drops[-1] - drops[0] if drops else None}
