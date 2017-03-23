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


def test_distance_between_simple(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(10, 0))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b2.teleport((0, 3))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(1)

    b2.teleport((0, 1.9))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b2.teleport((0.5, 0.5))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-1.5)

    b2.teleport((0, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-1)


def test_distance_between_simple2(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(10, 0))
    p = shyphe.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)])
    b2.add_shape(p)

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    c.position = (1, 0)
    b2.teleport((4, 0))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    c.position = (0, 0)
    b2.teleport((0, 3))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    x = (1 + 2 ** -0.5) * 2 ** -0.5

    b2.teleport((x - 0.0001, x - 0.0001))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-(0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((x + 0.0001, x + 0.0001))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx((0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((0, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-1 - 2 ** -0.5)


def test_distance_between_rotated(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(10, 0))
    p = shyphe.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)])
    b2.add_shape(p)

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b1.teleport((0, 0))
    b2.teleport((0, 3))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    x = (1 + 2 ** -0.5) * 2 ** -0.5

    b2.teleport((x - 0.0001, x - 0.0001))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-(0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((x + 0.0001, x + 0.0001))

    assert shyphe.distance_between(p, b2, c, b1).distance == pytest.approx((0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((0, 0))

    assert shyphe.distance_between(c, b1, p, b2).distance == pytest.approx(-1 - 2 ** -0.5)


def test_circle_polygon_horizontal(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(2, 0))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(10, 0), velocity=(-6, 0))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))


def test_circle_polygon_vertical(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, -6))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))


def test_circle_polygon_vertical_near_hit(shyphe):
    b1 = shyphe.Body(position=(2, 0), velocity=(0, 2))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, -6))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.125, 1e-5)


def test_circle_polygon_vertical_near_miss(shyphe):
    b1 = shyphe.Body(position=(2.001, 0), velocity=(0, 2))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, -6))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1


def test_circle_polygon_no_collision_opposite_dir(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 10), velocity=(0, 6))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0


def test_circle_polygon_no_collision_parallel(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(2, 0))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 10), velocity=(2, 0))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0


def test_circle_polygon_out_of_time(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(0, 2))
    c = shyphe.Circle(radius=1)
    b1.add_shape(c)

    b2 = shyphe.Body(position=(0, 100), velocity=(0, -6))
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = shyphe.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0
