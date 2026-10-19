#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run portable ranging tests; optionally compare the matching node firmware tree."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--node", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[3]
common = ["-Wall", "-Wextra", "-Werror", "-UNDEBUG", "-fsanitize=address,undefined"]
cc = os.environ.get("CC", "clang")
cxx = os.environ.get("CXX", "clang++")
with tempfile.TemporaryDirectory(prefix="twr-tests-") as directory:
    output = Path(directory) / "test"

    def check(compiler, sources, includes, standard):
        command = [compiler, standard, *common]
        command += ["-I" + str(path) for path in includes]
        command += [str(path) for path in sources]
        subprocess.run(command + ["-o", str(output)], check=True)
        subprocess.run([str(output)], check=True)
    ranging = root / "libraries/AP_Ranging"
    check(cc, [ranging / "tests/test_twr_protocol.c"], [ranging], "-std=c11")
    dw1000 = root / "modules/libdw1000"
    check(cc, [ranging / "tests/test_radio_diag.c",
               dw1000 / "src/libdw1000.c", dw1000 / "src/libdw1000Spi.c"],
          [ranging, dw1000 / "inc"], "-std=c11")
    check(cc, [ranging / "tests/test_dw1000_recovery.c",
               dw1000 / "src/libdw1000.c", dw1000 / "src/libdw1000Spi.c"],
          [ranging, dw1000 / "inc"], "-std=c11")
    check(cxx, [root / "libraries/AP_Beacon/tests/test_ranging_solver.cpp"], [], "-std=c++11")
    if args.node:
        node = args.node.resolve()
        assert (ranging / "twr_protocol.h").read_bytes() == (node / "inc/twr_protocol.h").read_bytes()
        assert (ranging / "tests/test_twr_protocol.c").read_bytes() == (node / "test/host/test_twr_protocol.c").read_bytes()
        for name in ("twr_radio_diag.h", "dw1000_radio_diag.h"):
            assert (ranging / name).read_bytes() == (node / "inc" / name).read_bytes()
        assert (ranging / "tests/test_radio_diag.c").read_bytes() == (node / "test/host/test_radio_diag.c").read_bytes()
        check(cc, [node / "test/host/test_radio_diag.c", node / "vendor/libdw1000/src/libdw1000.c",
                   node / "vendor/libdw1000/src/libdw1000Spi.c"],
              [node / "inc", node / "vendor/libdw1000/inc"], "-std=c11")
        check(cc, [node / "test/host/test_twr_protocol.c"], [node / "inc"], "-std=c11")
        check(cc, [node / "test/host/test_node_radio_diag.c", node / "vendor/libdw1000/src/libdw1000.c",
                   node / "vendor/libdw1000/src/libdw1000Spi.c"],
              [node / "test/host/radio_stubs", node / "inc", node / "src", node / "vendor/libdw1000/inc"], "-std=c11")
        check(cc, [node / "test/host/test_console_input.c"], [node / "inc"], "-std=c11")
        check(cc, [node / "test/host/test_cfg.c", node / "src/cfg.c"],
              [node / "test/host/stubs", node / "inc"], "-std=c11")
print("All portable ranging tests passed")
