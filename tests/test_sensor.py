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


def test_clone(shyphe):
    s = shyphe.ActiveRadar(power=50, sensitivity=2)
    s2 = s.clone()

    assert s.power == s2.power
    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)

    s = shyphe.PassiveRadar(sensitivity=3)
    s2 = s.clone()

    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)

    s = shyphe.PassiveThermal(sensitivity=4)
    s2 = s.clone()

    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)


def test_body_sensor(shyphe):
    s = shyphe.ActiveRadar(power=50, sensitivity=2)
    b = shyphe.Body()

    assert b.max_sensor_range == 0

    b.add_sensor(s)

    assert b.max_sensor_range == 625
    assert b.sensors[0] is s
    assert list(b.sensors) == [s]

    b.remove_sensor(s)

    assert b.max_sensor_range == 0


def test_perf(shyphe):
    s = shyphe.ActiveRadar(power=50, sensitivity=2)

    assert s.max_range == 625

    s.perf = 0.5

    assert s.max_range == 625 / 2

    s = shyphe.PassiveRadar(sensitivity=3)

    assert s.max_range == 50 / 3

    s.perf = 0.5

    assert s.max_range == 50 / 6

    s = shyphe.PassiveThermal(sensitivity=3)

    assert s.max_range == 2500 / 3

    s.perf = 0.5

    assert s.max_range == 2500 / 6
