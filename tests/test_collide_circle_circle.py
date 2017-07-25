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


def test_distance_between_circle_circle(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(10, 0))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    db = shyphe.distance_between(c1, b1, c2, b2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, 0)
    assert db.b_point.as_tuple() == (9, 0)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    db = shyphe.distance_between(c1, b1, c2, b2)

    assert db.distance == pytest.approx(1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (3, 0)

    b2.teleport((3, 0))

    db = shyphe.distance_between(c1, b1, c2, b2)

    assert db.distance == pytest.approx(0)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (2, 0)

    b2.teleport((2, 0))

    db = shyphe.distance_between(c1, b1, c2, b2)
    assert db.distance == pytest.approx(-1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (1, 0)

    b2.teleport((1, 0))

    db = shyphe.distance_between(c1, b1, c2, b2)
    assert db.distance == pytest.approx(-2)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (0, 0)

    b2.teleport((1, 1))

    assert shyphe.distance_between(c1, b1, c2, b2).distance == pytest.approx(-1)

    b1.teleport((0, 0))

    assert shyphe.distance_between(c1, b1, c2, b2).distance == pytest.approx(2 ** 0.5 - 2)

    b2.teleport((2, 2))

    assert shyphe.distance_between(c1, b1, c2, b2).distance == pytest.approx(8 ** 0.5 - 2)

    b2.teleport((3, 3))

    assert shyphe.distance_between(c1, b1, c2, b2).distance == pytest.approx(18 ** 0.5 - 2)


def test_circle_circle_horizontal(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(2, 0))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(10, 0), velocity=(-6, 0))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    coll = shyphe.collide_shapes(c1, b1, c2, b2, 2, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))


def test_circle_circle_vertical(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, -6))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    coll = shyphe.collide_shapes(c1, b1, c2, b2, 2, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))


def test_circle_circle_no_collision_opposite_dir(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, 6))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    coll = shyphe.collide_shapes(c1, b1, c2, b2, 1, False)
    assert coll.time == -1.0


def test_circle_circle_no_collision_parallel(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(2, 0))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(0, 10), velocity=(2, 0))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    coll = shyphe.collide_shapes(c1, b1, c2, b2, 1, False)
    assert coll.time == -1.0


def test_circle_circle_out_of_time(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c1 = shyphe.Circle(radius=1, mass=1)
    b1.add_shape(c1)

    b2 = shyphe.Body(position=(0, 100), velocity=(0, -6))
    c2 = shyphe.Circle(radius=1, mass=1)
    b2.add_shape(c2)

    coll = shyphe.collide_shapes(c1, b1, c2, b2, 1, False)
    assert coll.time == -1.0
