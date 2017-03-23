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


def test_corners(shyphe):
    aabb = shyphe.AABB(-1, 2, 0, 5)

    assert aabb.center.as_tuple() == (0.5, 2.5)
    assert aabb.topleft.as_tuple() == (-1, 5)
    assert aabb.topright.as_tuple() == (2, 5)
    assert aabb.bottomleft.as_tuple() == (-1, 0)
    assert aabb.bottomright.as_tuple() == (2, 0)


def test_construct(shyphe):
    x = shyphe.AABB(-1, 2, 0, 5)
    y = shyphe.AABB((0.5, 2.5), 3, 5)
    z = shyphe.AABB((-1, 5), (2, 0))

    assert x.as_tuple() == y.as_tuple() == z.as_tuple()


def test_aabb_str(shyphe):
    assert str(shyphe.AABB(-1, 2, 0, 5)) == "AABB(-1 - 2, 0 - 5)"
    assert repr(shyphe.AABB(-1, 2, 0, 5)) == "AABB(-1, 2, 0, 5)"
