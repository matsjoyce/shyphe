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


def test_distance_between_square_square(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, b1, p2, b2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, 0)
    assert db.b_point.as_tuple() == (9, 0)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    db = physics.distance_between(p1, b1, p2, b2)

    assert db.distance == pytest.approx(1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (3, 0)

    b2.teleport((3, 0))

    db = physics.distance_between(p1, b1, p2, b2)

    assert db.distance == pytest.approx(0)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (2, 0)

    b2.teleport((2, 0))

    db = physics.distance_between(p1, b1, p2, b2)
    assert db.distance == pytest.approx(-1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (1, 0)

    b2.teleport((1, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-2)

    b2.teleport((1, 1))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-1)

    b1.teleport((0, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-1)

    b2.teleport((2, 2))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(0)

    b2.teleport((3, 3))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(2 ** 0.5)


def test_distance_between_square_triangle(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(-1, -1), (1, -1), (0, 1)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, b1, p2, b2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, -1)
    assert db.b_point.as_tuple() == (9, -1)

    db = physics.distance_between(p2, b2, p1, b1)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((-1, 0))
    assert db.a_point.as_tuple() == (9, -1)
    assert db.b_point.as_tuple() == (1, -1)

    p1.position = (1, 0)
    b2.teleport((4, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(1)

    b2.teleport((3, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(0)

    b2.teleport((2, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-2 * 5 ** -0.5)

    b2.teleport((1, 0))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-4 * 5 ** -0.5)

    b2.teleport((1, 1))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-1)

    p1.position = (0, 0)

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(-1)

    b2.teleport((2, 2))

    assert physics.distance_between(p1, b1, p2, b2).distance == pytest.approx(0)


def test_distance_between_triangle_triangle(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(0, 0), (1, 1), (1, 0)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(1.5, 1), (2, 0), (-1, 0.5)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, b1, p2, b2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, 0.5)
    assert db.b_point.as_tuple() == (9, 0.5)

    db = physics.distance_between(p2, b2, p1, b1)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((-1, 0))
    assert db.a_point.as_tuple() == (9, 0.5)
    assert db.b_point.as_tuple() == (1, 0.5)


def test_square_square_horizontal(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0), velocity=(-6, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))


def test_square_square_vertical(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))


def test_square_square_vertical_near_hit(physics):
    b1 = physics.Body(position=(1.99999, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1.5, False)
    assert coll.time == pytest.approx(1)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((1.99999 / 2, 3))


def test_square_square_vertical_near_miss(physics):
    b1 = physics.Body(position=(2.001, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1, False)
    assert coll.time == -1


def test_square_square_no_collision_opposite_dir(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, 6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1, False)
    assert coll.time == -1.0


def test_square_square_no_collision_parallel(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(2, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1, False)
    assert coll.time == -1.0


def test_square_square_out_of_time(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 100), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, b1, p2, b2, 1, False)
    assert coll.time == -1.0
