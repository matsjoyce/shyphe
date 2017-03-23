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
    c = shyphe.Circle(radius=5, mass=10)

    assert c.mass == 10
    assert c.radius == 5

    c.mass = 50

    assert c.mass == 50

    c2 = c.clone()

    assert c2.mass == 50
    assert c2.radius == 5
    assert type(c2) is type(c)

    c2.mass = 55
    c2.radius = 10

    assert c.mass == 50
    assert c2.mass == 55
    assert c.radius == 5
    assert c2.radius == 10


def test_aabb(shyphe):
    c = shyphe.Circle(radius=5, mass=10)

    assert c.can_collide()
    assert c.aabb(0).as_tuple() == (-5, 5, -5, 5)
    assert c.aabb(1).as_tuple() == (-5, 5, -5, 5)

    c = shyphe.Circle(radius=100, mass=10)

    assert c.aabb(0).as_tuple() == (-100, 100, -100, 100)
    assert c.aabb(1).as_tuple() == (-100, 100, -100, 100)


def test_moi(shyphe):
    c = shyphe.Circle(radius=5, mass=10)

    assert c.moment_of_inertia == 125

    c = shyphe.Circle(radius=10, mass=10)

    assert c.moment_of_inertia == 500

    c = shyphe.Circle(radius=10, mass=5)

    assert c.moment_of_inertia == 250
