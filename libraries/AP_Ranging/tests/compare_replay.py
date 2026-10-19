#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Compare original/replayed EKF3 records by time when replay emits extra log samples."""
import argparse

from pymavlink import DFReader

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log", help="Replay output containing both original and replayed cores")
parser.add_argument("--allow-incomplete-tail", action="store_true",
                    help="Permit one final source logging timestamp without its closing replay frame")
args = parser.parse_args()
names = ["XKF1", "XKF2", "XKF3", "XKF4", "XKF0", "XKFS", "XKQ", "XKFD", "XKV1", "XKV2", "XKY0", "XKY1"]
reader = DFReader.DFReader_binary(args.log)
original = {}
replayed = {}
while True:
    message = reader.recv_match(type=names)
    if message is None:
        break
    if not hasattr(message, "C"):
        continue
    core = message.C
    values = message.to_dict()
    values["C"] = core % 100
    key = (message.get_type(), core % 100, message.TimeUS)
    (original if core < 100 else replayed)[key] = values
if not original or not replayed:
    raise SystemExit("Missing original or replayed EKF records")
common = original.keys() & replayed.keys()
missing = original.keys() - replayed.keys()
last_replay = max(key[2] for key in replayed)
tail_times = {key[2] for key in missing}
if missing and not (args.allow_incomplete_tail and len(tail_times) == 1 and
                    0 < next(iter(tail_times)) - last_replay <= 200000):
    raise SystemExit("Unmatched original records: %s" % sorted(missing)[:20])
for key in sorted(common):
    if original[key] != replayed[key]:
        raise SystemExit("EKF mismatch at %s: %s != %s" % (key, original[key], replayed[key]))
print("%u original EKF3 records match exactly; %u trailing records lack a closing frame" % (len(common), len(missing)))
