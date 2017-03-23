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


def test_clone(shyphe):
    p = shyphe.Polygon(points=[(0, 0), (1, 1), (1, 0)], mass=10)

    assert p.mass == 10
    assert list(p.points) == [(0, 0), (1, 1), (1, 0)]

    p.mass = 50

    assert p.mass == 50

    p2 = p.clone()

    assert p2.mass == 50
    assert list(p2.points) == [(0, 0), (1, 1), (1, 0)]
    assert type(p2) is type(p)

    p2.mass = 55

    assert p.mass == 50
    assert p2.mass == 55


def test_bad_shaped(shyphe):
    # concave
    with pytest.raises(RuntimeError):
        shyphe.Polygon(points=[(0, 1), (1, 1), (1, 0), (0.8, 0.8)], mass=10)

    # not enough points
    with pytest.raises(RuntimeError):
        shyphe.Polygon(points=[(0, 0), (1, 1)])


def test_normalization(shyphe):
    p1 = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    p2 = shyphe.Polygon(points=[(1, -1), (1, 1), (-1, 1), (-1, -1)])

    assert list(p1.points) == list(p2.points) == [shyphe.Vec(-1, -1), shyphe.Vec(-1, 1),
                                                  shyphe.Vec(1, 1), shyphe.Vec(1, -1)]


def test_degenerate(shyphe):
    p1 = shyphe.Polygon(points=[(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)])

    assert list(p1.points) == [shyphe.Vec(-1, -1), shyphe.Vec(-1, 1), shyphe.Vec(1, 1), shyphe.Vec(1, -1)]


def test_aabb(shyphe):
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])

    assert p.can_collide()
    assert p.aabb(0).as_tuple() == (-1, 1, -1, 1)
    assert p.aabb(shyphe.to_rad(45)).as_tuple() == pytest.approx((-2 ** 0.5, 2 ** 0.5, -2 ** 0.5, 2 ** 0.5))


def test_moi(shyphe):
    p = shyphe.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)], mass=5)

    assert p.moment_of_inertia == 10 / 3

    p = shyphe.Polygon(points=[(0, 0), (1, 1), (1, 0)], mass=10)

    assert p.moment_of_inertia == 20 / 3
