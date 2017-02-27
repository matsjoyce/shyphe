#!/bin/python3
import pytest
import argparse
import subprocess
import shutil
import pathlib
import sys
import os

parser = argparse.ArgumentParser()
parser.add_argument("--coverage", action="store_true")
args, pytestargs = parser.parse_known_args()

os.chdir(pathlib.Path(__file__).parent)
subprocess.check_call(["ninja"], cwd="build")

if args.coverage:
    from build.coverage import physics
    sys.modules["physics"] = physics

    for path in pathlib.Path("build").glob("**/*.gcda"):
        path.unlink()
else:
    from build import physics
    sys.modules["physics"] = physics

pytestargs.extend(["-v", "--durations", "3"])

exit_code = pytest.main(pytestargs, [])

if exit_code:
    exit(exit_code)
