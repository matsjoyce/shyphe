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


def test_active_radar(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    b1.add_sensor(shyphe.ActiveRadar(power=50, sensitivity=1))

    b2 = shyphe.Body(position=(10, 10))
    b2.add_shape(shyphe.MassShape(radar_cross_section=20))

    b3 = shyphe.Body(position=(10, -10))
    b3.add_shape(shyphe.MassShape(radar_emissions=10, thermal_emissions=10))

    w = shyphe.World(1)
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame()
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.signature.as_tuple() == (0, 0, 20)
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == shyphe.Side.neutral


def test_out_of_range(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    s = shyphe.ActiveRadar(power=5, sensitivity=1)
    b1.add_sensor(s)

    b2 = shyphe.Body(position=(s.max_range + 0.1, 0))
    b2.add_shape(shyphe.MassShape(radar_cross_section=20))

    w = shyphe.World(1)
    w.add_body(b1)
    w.add_body(b2)

    w.begin_frame()
    w.end_frame()

    assert len(b1.sensor_view) == 0


def test_passive_radar(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    b1.add_sensor(shyphe.PassiveRadar(sensitivity=1))

    b2 = shyphe.Body(position=(10, 10))
    b2.add_shape(shyphe.MassShape(radar_emissions=25))

    b3 = shyphe.Body(position=(10, -10))
    b3.add_shape(shyphe.MassShape(radar_cross_section=10, thermal_emissions=10))

    w = shyphe.World(1)
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame()
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.signature.as_tuple() == (25, 0, 0)
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == shyphe.Side.unknown


def test_passive_thermal(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    b1.add_sensor(shyphe.PassiveThermal(sensitivity=1))

    b2 = shyphe.Body(position=(10, 10))
    b2.add_shape(shyphe.MassShape(thermal_emissions=15))

    b3 = shyphe.Body(position=(10, -10))
    b3.add_shape(shyphe.MassShape(radar_cross_section=10, radar_emissions=10))

    w = shyphe.World(1)
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame()
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.signature.as_tuple() == (0, 15, 0)
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == shyphe.Side.unknown


def test_tracking(shyphe):
    b = shyphe.Body(position=(0, 0), side=1)
    b.add_sensor(shyphe.ActiveRadar(power=50, sensitivity=2))
    b.add_shape(shyphe.MassShape(radar_cross_section=50, mass=1))

    b2 = shyphe.Body(position=(10, 0), velocity=(5, 0), side=1)
    b2.add_sensor(shyphe.ActiveRadar(power=50, sensitivity=2))
    b2.add_shape(shyphe.MassShape(radar_cross_section=50, mass=1))

    w = shyphe.World(1)
    w.add_body(b)
    w.add_body(b2)

    w.begin_frame()
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    sr = b.sensor_view[0]
    assert sr.side == shyphe.Side.friendly
    assert sr.position.as_tuple() == (10, 0)
    assert not sr.velocity
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == shyphe.Side.friendly
    assert sr.position.as_tuple() == (-10, 0)
    assert not sr.velocity
    assert sr.body == b

    assert b2.position.as_tuple() == (15, 0)

    b2.change_side(2)

    w.begin_frame()
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    assert b.sensor_view[0] == b.sensor_view[0]
    assert b.sensor_view[0] != b2.sensor_view[0]

    sr = b.sensor_view[0]
    assert sr.side == shyphe.Side.enemy
    assert sr.position.as_tuple() == (15, 0)
    assert sr.velocity.as_tuple() == (5, 0)
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == shyphe.Side.enemy
    assert sr.position.as_tuple() == (-15, 0)
    assert sr.velocity.as_tuple() == (-5, 0)
    assert sr.body == b

    assert b2.position.as_tuple() == (20, 0)

    b2.teleport((-30, 60))

    w.begin_frame()
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    sr = b.sensor_view[0]
    assert sr.side == shyphe.Side.enemy
    assert sr.position.as_tuple() == (-30, 60)
    assert sr.velocity.as_tuple() == (-55, 60)
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == shyphe.Side.enemy
    assert sr.position.as_tuple() == (30, -60)
    assert sr.velocity.as_tuple() == (55, -60)
    assert sr.body == b
