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


def test_complex_collision(shyphe):
    a = shyphe.Body(position=(0, 0), velocity=(30, 40))
    a_ms = shyphe.MassShape(mass=10)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(2, 0), velocity=(-30, 0))
    b_ms = shyphe.MassShape(mass=10)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (0, 0), (b.position - a.position).norm())

    cr = shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))

    assert cr.impulse.as_tuple() == (-600, 0)
    assert cr.closing_velocity.as_tuple() == (-60, 0)


def test_same_direction_vertical(shyphe):
    a = shyphe.Body(position=(0, 2), velocity=(0, -30))
    a_ms = shyphe.MassShape(mass=10)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(0, 0), velocity=(0, -20))
    b_ms = shyphe.MassShape(mass=10)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (0, 0), (b.position - a.position).norm())

    cr = shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))

    assert cr.impulse.as_tuple() == (0, 100)
    assert cr.closing_velocity.as_tuple() == (0, 10)


def test_same_direction_horizontal(shyphe):
    a = shyphe.Body(position=(0, 0), velocity=(-20, 0))
    a_ms = shyphe.MassShape(mass=10)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(2, 0), velocity=(-30, 0))
    b_ms = shyphe.MassShape(mass=10)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (0, 0), (b.position - a.position).norm())

    cr = shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))

    assert cr.impulse.as_tuple() == (-100, 0)
    assert cr.closing_velocity.as_tuple() == (-10, 0)


def test_rotating(shyphe):
    a = shyphe.Body(position=(0, 0), angular_velocity=1)
    a_ms = shyphe.Circle(mass=10, position=(0, 1), radius=1)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(2, 0), angular_velocity=-1)
    b_ms = shyphe.Circle(mass=10, position=(0, 1), radius=1)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (1, 1), (b.position - a.position).norm())

    cr = shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))

    assert cr.impulse.as_tuple() == pytest.approx((-12, 0))
    assert cr.closing_velocity.as_tuple() == (-2, 0)


def test_no_collision(shyphe):
    a = shyphe.Body(position=(0, 0), velocity=(-30, 0))
    a_ms = shyphe.MassShape(mass=10)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(2, 0), velocity=(-20, 0))
    b_ms = shyphe.MassShape(mass=10)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (0, 0), (b.position - a.position).norm())

    with pytest.raises(RuntimeError):
        shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))

    a = shyphe.Body(position=(0, 0), angular_velocity=-1, velocity=(1, 0))
    a_ms = shyphe.Circle(mass=10, position=(0, 1), radius=1)
    a.add_shape(a_ms)
    b = shyphe.Body(position=(2, 0), angular_velocity=1)
    b_ms = shyphe.Circle(mass=10, position=(0, 1), radius=1)
    b.add_shape(b_ms)

    col = shyphe.CollisionTimeResult(0, (1, 1), (b.position - a.position).norm())

    with pytest.raises(RuntimeError):
        shyphe.collision_result(col, a, b, shyphe.CollisionParameters(1))
