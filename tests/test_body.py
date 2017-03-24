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


def test_add_shape(shyphe):
    b = shyphe.Body()
    m = shyphe.MassShape(mass=10)
    b.add_shape(m)

    assert b.mass == 10
    assert b.shapes[0] is m
    assert list(b.shapes) == [m]


def test_remove_shape(shyphe):
    b = shyphe.Body()
    c = shyphe.MassShape(mass=10)
    b.add_shape(c)
    b.remove_shape(c)

    assert b.mass == 0


def test_body_distance(shyphe):
    b1 = shyphe.Body(position=(0, 0))
    b1.add_shape(shyphe.Circle(radius=1, position=(1, 0)))
    b1.add_shape(shyphe.Circle(radius=1, position=(-1, 0)))
    b1.add_shape(shyphe.MassShape(position=(5, 0)))

    b2 = shyphe.Body(position=(10, 0))
    b2.add_shape(shyphe.Circle(radius=1, position=(0, 1)))
    b2.add_shape(shyphe.Circle(radius=1, position=(-1, 0)))
    b2.add_shape(shyphe.MassShape(position=(-5, 0)))

    assert b1.distance_between(b2) == 6


def test_body_collide(shyphe):
    b1 = shyphe.Body(position=(0, 0), velocity=(1, 0))
    b1.add_shape(shyphe.Circle(radius=1, position=(1, 0)))
    b1.add_shape(shyphe.Circle(radius=1, position=(-1, 0)))
    b1.add_shape(shyphe.MassShape(position=(5, 0)))

    b2 = shyphe.Body(position=(10, 0), velocity=(-5, 0))
    b2.add_shape(shyphe.Circle(radius=1, position=(0, 1)))
    b2.add_shape(shyphe.Circle(radius=1, position=(-1, 0)))
    b2.add_shape(shyphe.MassShape(position=(-5, 0)))

    colr, a, b = b1.collide(b2, 2, False)

    assert colr.time == pytest.approx(1.0)


def test_local_linear_acceleration(shyphe):
    b = shyphe.Body()
    c = shyphe.MassShape(mass=1)
    b.add_shape(c)
    b.apply_local_force((1, 0), (0, 0))

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0)
    assert b.angular_velocity == 0

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (2, 0)
    assert b.angular_velocity == 0

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 0

    b.clear_local_forces()

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 0


def test_global_linear_acceleration(shyphe):
    b = shyphe.Body()
    c = shyphe.MassShape(mass=1)
    b.add_shape(c)
    b.apply_global_force((1, 0), (0, 0))

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0)
    assert b.angular_velocity == 0

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (2, 0)
    assert b.angular_velocity == 0

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 0

    b.clear_global_forces()

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 0


def test_local_rotational_acceleration(shyphe):
    b = shyphe.Body()
    c = shyphe.MassShape(mass=1)
    b.add_shape(c)
    b.apply_local_force((1, 0), (0, 1))

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0)
    assert b.angular_velocity == 1
    assert b.angle == 1

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0) + shyphe.Vec(1, 0).rotate(1)
    assert b.angular_velocity == 2

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0) + shyphe.Vec(1, 0).rotate(1) + shyphe.Vec(1, 0).rotate(3)
    assert b.angular_velocity == 3

    b.clear_local_forces()

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0) + shyphe.Vec(1, 0).rotate(1) + shyphe.Vec(1, 0).rotate(3)
    assert b.angular_velocity == 3


def test_global_rotational_acceleration(shyphe):
    b = shyphe.Body()
    c = shyphe.MassShape(mass=1)
    b.add_shape(c)
    b.apply_global_force((1, 0), (0, 1))

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (1, 0)
    assert b.angular_velocity == 1

    b.update_position(1)
    b.update_velocity(1)

    assert b.velocity == (2, 0)
    assert b.angular_velocity == 2

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 3

    b.clear_global_forces()

    b.update_velocity(1)
    b.update_position(1)

    assert b.velocity == (3, 0)
    assert b.angular_velocity == 3


def test_aabb(shyphe):
    b = shyphe.Body()
    c = shyphe.Circle(radius=1, mass=1)
    b.add_shape(c)

    assert b.aabb(0).as_tuple() == (-1, 1, -1, 1)
    assert b.aabb(1).as_tuple() == (-1, 1, -1, 1)
    assert b.aabb(0.01).as_tuple() == (-1, 1, -1, 1)

    c.position = (1, 0)

    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (0, 2, -1, 1)

    b.apply_impulse((1, 0), (0, 0))

    assert b.velocity.as_tuple() == (1, 0)

    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (0, 3, -1, 1)

    b.apply_impulse((-1, -1), (0, 0))
    b.apply_impulse((0, 1), (3 / 2 * shyphe.to_rad(45), 0))

    assert b.angular_velocity == -shyphe.to_rad(45)
    assert b.velocity.as_tuple() == (0, 0)
    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (2 ** -0.5 - 1, 2, -1, 2 ** -0.5 + 1)
    assert b.aabb(2).as_tuple() == pytest.approx((-1, 2, -1, 2))
    assert b.aabb(3).as_tuple() == (-2 ** -0.5 - 1, 2, -1, 2)
    assert b.aabb(4).as_tuple() == pytest.approx((-2, 2, -1, 2))

    b.apply_impulse((1, 0), (0, 0))

    assert b.angular_velocity == -shyphe.to_rad(45)
    assert b.velocity.as_tuple() == (1, 0)
    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    # assert b.aabb(1).as_tuple() == (2 ** -0.5 - 1, 2, -1, 2 ** -0.5 + 1)
    # assert b.aabb(2).as_tuple() == pytest.approx((-1, 3, -1, 2))
    # assert b.aabb(3).as_tuple() == (-2 ** -0.5 - 1, 2, -1, 2)
    # assert b.aabb(4).as_tuple() == pytest.approx((-2, 4, -1, 2))
