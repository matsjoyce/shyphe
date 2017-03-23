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

def test_clone(physics):
    m = physics.MassShape(mass=45)

    assert m.mass == 45

    m.mass = 50

    assert m.mass == 50

    m2 = m.clone()

    assert m2.mass == 50
    assert type(m2) is type(m)

    m2.mass = 55

    assert m.mass == 50
    assert m2.mass == 55


def test_aabb(physics):
    m = physics.MassShape(mass=45)

    assert not m.can_collide()
    assert m.aabb(0).as_tuple() == (0, 0, 0, 0)
    assert m.aabb(1).as_tuple() == (0, 0, 0, 0)


def test_moi(physics):
    m = physics.MassShape(mass=45)

    assert m.moment_of_inertia == 1
