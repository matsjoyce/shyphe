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

import math
import pytest


def map_round(a):
    return tuple(round(i, 3) for i in a)


def test_construct(shyphe):
    assert shyphe.Vec(1, 2).as_tuple() == (1, 2)
    assert shyphe.Vec() == (0, 0)


def test_distance_to(shyphe):
    assert shyphe.Vec(1, 1).distance_to((5, 4)) == 5
    assert shyphe.Vec(1, 1).distance_to((1, 2)) == 1
    assert shyphe.Vec(1, 1).distance_to((1, 0)) == 1
    assert shyphe.Vec(1, 1).distance_to((2, 1)) == 1
    assert shyphe.Vec(1, 1).distance_to((0, 1)) == 1
    assert shyphe.Vec(1, 1).distance_to((0, 0)) == 2 ** 0.5


def test_abs(shyphe):
    assert shyphe.Vec(1, 1).abs() == 2 ** 0.5
    assert shyphe.Vec(0, 1).abs() == 1
    assert shyphe.Vec(1, 0).abs() == 1
    assert shyphe.Vec(0, -1).abs() == 1
    assert shyphe.Vec(-1, 0).abs() == 1
    assert shyphe.Vec(2, 0).abs() == 2


def test_squared(shyphe):
    assert shyphe.Vec(1, 1).squared() == 2
    assert shyphe.Vec(0, 1).squared() == 1
    assert shyphe.Vec(1, 0).squared() == 1
    assert shyphe.Vec(0, -1).squared() == 1
    assert shyphe.Vec(-1, 0).squared() == 1
    assert shyphe.Vec(2, 0).squared() == 4


def test_reflect(shyphe):
    assert shyphe.Vec(1, 1).reflect((0, 1)).as_tuple() == (-1, 1)
    assert shyphe.Vec(1, 1).reflect((1, 0)).as_tuple() == (1, -1)
    assert shyphe.Vec(1, 1).reflect((1, 1)).as_tuple() == (1, 1)
    assert shyphe.Vec(1, 1).reflect((-1, 1)).as_tuple() == (-1, -1)


def test_norm(shyphe):
    assert shyphe.Vec(1, 1).norm().as_tuple() == pytest.approx((2 ** -0.5, 2 ** -0.5))
    assert shyphe.Vec(1, 0).norm().as_tuple() == (1, 0)
    assert shyphe.Vec(-3, 4).norm().as_tuple() == (-0.6, 0.8)


def test_operators(shyphe):
    assert shyphe.Vec(1, 1) + shyphe.Vec(1, 1) == shyphe.Vec(2, 2)
    assert shyphe.Vec(1, 1) - shyphe.Vec(1, 1) == shyphe.Vec(0, 0)
    assert shyphe.Vec(1, 5) + shyphe.Vec(-1, 1) == shyphe.Vec(0, 6)
    assert shyphe.Vec(1, 5) - shyphe.Vec(-1, 1) == shyphe.Vec(2, 4)

    assert shyphe.Vec(1, 5) * 5 == shyphe.Vec(5, 25)
    assert 5 * shyphe.Vec(1, 5) == shyphe.Vec(5, 25)
    assert shyphe.Vec(25, 5) / 5 == shyphe.Vec(5, 1)

    assert -shyphe.Vec(25, 5) == shyphe.Vec(-25, -5)


def test_compare(shyphe):
    assert shyphe.Vec(1, 1) == shyphe.Vec(1, 1)
    assert not (shyphe.Vec(1, 1) != shyphe.Vec(1, 1))
    assert not (shyphe.Vec(1, 2) == shyphe.Vec(1, 1))
    assert shyphe.Vec(1, 2) != shyphe.Vec(1, 1)

    assert shyphe.Vec(1, 1) < shyphe.Vec(2, 2)
    assert shyphe.Vec(1, 1) <= shyphe.Vec(2, 2)
    assert not (shyphe.Vec(1, 1) > shyphe.Vec(2, 2))
    assert not (shyphe.Vec(1, 1) >= shyphe.Vec(2, 2))

    assert not (shyphe.Vec(1, 1) < shyphe.Vec(1, 1))
    assert shyphe.Vec(1, 1) <= shyphe.Vec(1, 1)
    assert not (shyphe.Vec(1, 1) > shyphe.Vec(1, 1))
    assert shyphe.Vec(1, 1) >= shyphe.Vec(1, 1)

    assert shyphe.Vec(1, 1) < shyphe.Vec(1, 2)
    assert shyphe.Vec(1, 1) <= shyphe.Vec(1, 2)
    assert not (shyphe.Vec(1, 1) > shyphe.Vec(1, 2))
    assert not (shyphe.Vec(1, 1) >= shyphe.Vec(1, 2))

    assert not (shyphe.Vec(1, 2) < shyphe.Vec(1, 1))
    assert not (shyphe.Vec(1, 2) <= shyphe.Vec(1, 1))
    assert shyphe.Vec(1, 2) > shyphe.Vec(1, 1)
    assert shyphe.Vec(1, 2) >= shyphe.Vec(1, 1)


def test_dot(shyphe):
    assert shyphe.Vec(0, 1).dot((1, 0)) == 0
    assert shyphe.Vec(1, 1).dot((1, 0)) == 1
    assert shyphe.Vec(1, 1).dot((1, 1)) == 2
    assert shyphe.Vec(-1, -1).dot((1, 1)) == -2


def test_proj_rej(shyphe):
    assert shyphe.Vec(5, 0).proj((1, 1)).as_tuple() == (2.5, 2.5)
    assert shyphe.Vec(5, 0).rej((1, 1)).as_tuple() == (2.5, -2.5)
    assert shyphe.Vec(5, 9).proj((2, 4)) + shyphe.Vec(5, 9).rej((2, 4)) == shyphe.Vec(5, 9)


def test_from_bearing(shyphe):
    assert (shyphe.Vec.from_bearing(shyphe.Vec(5, 9).bearing()).as_tuple()
            == pytest.approx(shyphe.Vec(5, 9).norm().as_tuple()))
    assert (shyphe.Vec.from_bearing(shyphe.Vec(1, 0).bearing()).as_tuple()
            == pytest.approx(shyphe.Vec(1, 0).norm().as_tuple()))
    assert (shyphe.Vec.from_bearing(shyphe.Vec(0, 1).bearing()).as_tuple()
            == pytest.approx(shyphe.Vec(0, 1).norm().as_tuple()))
    assert (shyphe.Vec.from_bearing(shyphe.Vec(-1, 0).bearing()).as_tuple()
            == pytest.approx(shyphe.Vec(-1, 0).norm().as_tuple()))
    assert (shyphe.Vec.from_bearing(shyphe.Vec(0, -1).bearing()).as_tuple()
            == pytest.approx(shyphe.Vec(0, -1).norm().as_tuple()))


def test_bearing_to(shyphe):
    assert shyphe.Vec(0, 0).bearing_to((1, 0)) == math.pi / 2
    assert shyphe.Vec(0, 0).bearing_to((-1, 0)) == -math.pi / 2
    assert shyphe.Vec(0, 0).bearing_to((0, 1)) == 0
    assert shyphe.Vec(0, 0).bearing_to((0, -1)) == math.pi


def test_vec_str(shyphe):
    assert str(shyphe.Vec(1, 3)) == "(1, 3)"
    assert repr(shyphe.Vec(1, 3)) == "Vec(1, 3)"


def test_python_conv(shyphe):
    v = shyphe.Vec(1.1, 2.1)

    assert len(v) == 2
    assert v[0] == v[-2] == 1.1
    assert v[1] == v[-1] == 2.1

    with pytest.raises(IndexError):
        v[2]

    with pytest.raises(IndexError):
        v[-3]

    assert list(v) == [1.1, 2.1]

    v[0] = 3

    assert v.as_tuple() == (3, 2.1)

    v[1] = 4

    assert v.as_tuple() == (3, 4)

    with pytest.raises(IndexError):
        v[2] = 1

    with pytest.raises(IndexError):
        v[-3] = 1

    with pytest.raises(ValueError):
        v.distance_to((1,))

    with pytest.raises(ValueError):
        v.distance_to((1, 2, 3))

    with pytest.raises(ValueError):
        v.distance_to((1, None))

    with pytest.raises(ValueError):
        v.distance_to((None, None))

    with pytest.raises(TypeError):
        v.distance_to([])

    assert shyphe.Vec(1, 0)
    assert shyphe.Vec(0, 1)
    assert not shyphe.Vec()


def test_rotate(shyphe):
    assert shyphe.Vec(1, 0).rotate(0).as_tuple() == pytest.approx((1, 0))
    assert shyphe.Vec(1, 0).rotate(math.pi).as_tuple() == pytest.approx((-1, 0))
    assert shyphe.Vec(0, 1).rotate(math.pi).as_tuple() == pytest.approx((0, -1))
    assert shyphe.Vec(2, 1).rotate(math.pi).as_tuple() == pytest.approx((-2, -1))
    assert shyphe.Vec(2, 1).rotate(math.pi / 2).as_tuple() == pytest.approx((1, -2))
    assert shyphe.Vec(2, 1).rotate(-math.pi / 2).as_tuple() == pytest.approx((-1, 2))
    assert shyphe.Vec(0, 1).rotate(math.pi / 4).as_tuple() == pytest.approx((2 ** -0.5, 2 ** -0.5))
    assert shyphe.Vec(1, 1).rotate(math.pi / 4).as_tuple() == pytest.approx((2 ** 0.5, 0))
