# shyphe - Stiff HIgh velocity PHysics Engine
# Copyright (C) 2017 Matthew Joyce matsjoyce@gmail.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import pytest
import sys
import pathlib
import subprocess


DIR = pathlib.Path(__file__).parent.parent


@pytest.fixture
def physics(request):
    return sys.modules["physics"]


def pytest_configure(config):
    if config.getoption("--coverage"):
        print("Removing old coverage files...", end=" ", flush=True)
        for path in (DIR / "build").glob("**/*.gcda"):
            path.unlink()
        print("done")
    print("Building physics module...", end=" ", flush=True)

    sys.path.insert(0, str(DIR))

    if not (DIR / "build").exists():
        (DIR / "build").mkdir()
        subprocess.check_call("cmake .. -G Ninja".split(), cwd=DIR / "build",
                              stdout=subprocess.DEVNULL)

    subprocess.check_call(["ninja"], cwd=DIR / "build", stdout=subprocess.DEVNULL)
    if config.getoption("--coverage"):
        from build.coverage import physics
        sys.modules["physics"] = physics
    else:
        from build import physics
        sys.modules["physics"] = physics
    sys.path.pop(0)

    print("done")


def pytest_unconfigure(config):
    if config.getoption("--coverage"):
        import physics

        physics.flush_coverage()

        print("Generating coverage report...", end=" ", flush=True)
        subprocess.check_call("lcov --capture --directory . --output-file .coverage.info --no-external".split(),
                              stdout=subprocess.DEVNULL)
        subprocess.check_call("genhtml .coverage.info --output-directory coveragehtml".split(),
                              stdout=subprocess.DEVNULL)
        pathlib.Path(".coverage.info").unlink()
        print("done")
