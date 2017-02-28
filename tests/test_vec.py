import physics
import math
import pytest


def map_round(a):
    return tuple(round(i, 3) for i in a)


def test_construct():
    assert physics.Vec(1, 2).as_tuple() == (1, 2)
    assert physics.Vec() == (0, 0)


def test_distance_to():
    assert physics.Vec(1, 1).distance_to((5, 4)) == 5
    assert physics.Vec(1, 1).distance_to((1, 2)) == 1
    assert physics.Vec(1, 1).distance_to((1, 0)) == 1
    assert physics.Vec(1, 1).distance_to((2, 1)) == 1
    assert physics.Vec(1, 1).distance_to((0, 1)) == 1
    assert physics.Vec(1, 1).distance_to((0, 0)) == 2 ** 0.5


def test_abs():
    assert physics.Vec(1, 1).abs() == 2 ** 0.5
    assert physics.Vec(0, 1).abs() == 1
    assert physics.Vec(1, 0).abs() == 1
    assert physics.Vec(0, -1).abs() == 1
    assert physics.Vec(-1, 0).abs() == 1
    assert physics.Vec(2, 0).abs() == 2


def test_squared():
    assert physics.Vec(1, 1).squared() == 2
    assert physics.Vec(0, 1).squared() == 1
    assert physics.Vec(1, 0).squared() == 1
    assert physics.Vec(0, -1).squared() == 1
    assert physics.Vec(-1, 0).squared() == 1
    assert physics.Vec(2, 0).squared() == 4


def test_norm():
    assert physics.Vec(1, 1).norm().as_tuple() == pytest.approx((2 ** -0.5, 2 ** -0.5))
    assert physics.Vec(1, 0).norm().as_tuple() == (1, 0)
    assert physics.Vec(-3, 4).norm().as_tuple() == (-0.6, 0.8)


def test_operators():
    assert physics.Vec(1, 1) + physics.Vec(1, 1) == physics.Vec(2, 2)
    assert physics.Vec(1, 1) - physics.Vec(1, 1) == physics.Vec(0, 0)
    assert physics.Vec(1, 5) + physics.Vec(-1, 1) == physics.Vec(0, 6)
    assert physics.Vec(1, 5) - physics.Vec(-1, 1) == physics.Vec(2, 4)

    assert physics.Vec(1, 5) * 5 == physics.Vec(5, 25)
    assert 5 * physics.Vec(1, 5) == physics.Vec(5, 25)
    assert physics.Vec(25, 5) / 5 == physics.Vec(5, 1)

    assert -physics.Vec(25, 5) == physics.Vec(-25, -5)


def test_compare():
    assert physics.Vec(1, 1) == physics.Vec(1, 1)
    assert not (physics.Vec(1, 1) != physics.Vec(1, 1))
    assert not (physics.Vec(1, 2) == physics.Vec(1, 1))
    assert physics.Vec(1, 2) != physics.Vec(1, 1)

    assert physics.Vec(1, 1) < physics.Vec(2, 2)
    assert physics.Vec(1, 1) <= physics.Vec(2, 2)
    assert not (physics.Vec(1, 1) > physics.Vec(2, 2))
    assert not (physics.Vec(1, 1) >= physics.Vec(2, 2))

    assert not (physics.Vec(1, 1) < physics.Vec(1, 1))
    assert physics.Vec(1, 1) <= physics.Vec(1, 1)
    assert not (physics.Vec(1, 1) > physics.Vec(1, 1))
    assert physics.Vec(1, 1) >= physics.Vec(1, 1)

    assert physics.Vec(1, 1) < physics.Vec(1, 2)
    assert physics.Vec(1, 1) <= physics.Vec(1, 2)
    assert not (physics.Vec(1, 1) > physics.Vec(1, 2))
    assert not (physics.Vec(1, 1) >= physics.Vec(1, 2))

    assert not (physics.Vec(1, 2) < physics.Vec(1, 1))
    assert not (physics.Vec(1, 2) <= physics.Vec(1, 1))
    assert physics.Vec(1, 2) > physics.Vec(1, 1)
    assert physics.Vec(1, 2) >= physics.Vec(1, 1)


def test_dot():
    assert physics.Vec(0, 1).dot((1, 0)) == 0
    assert physics.Vec(1, 1).dot((1, 0)) == 1
    assert physics.Vec(1, 1).dot((1, 1)) == 2
    assert physics.Vec(-1, -1).dot((1, 1)) == -2


def test_proj_rej():
    assert physics.Vec(5, 0).proj((1, 1)).as_tuple() == (2.5, 2.5)
    assert physics.Vec(5, 0).rej((1, 1)).as_tuple() == (2.5, -2.5)
    assert physics.Vec(5, 9).proj((2, 4)) + physics.Vec(5, 9).rej((2, 4)) == physics.Vec(5, 9)


def test_from_bearing():
    assert physics.Vec.from_bearing(physics.Vec(5, 9).bearing()).as_tuple() == pytest.approx(physics.Vec(5, 9).norm().as_tuple())
    assert physics.Vec.from_bearing(physics.Vec(1, 0).bearing()).as_tuple() == pytest.approx(physics.Vec(1, 0).norm().as_tuple())
    assert physics.Vec.from_bearing(physics.Vec(0, 1).bearing()).as_tuple() == pytest.approx(physics.Vec(0, 1).norm().as_tuple())
    assert physics.Vec.from_bearing(physics.Vec(-1, 0).bearing()).as_tuple() == pytest.approx(physics.Vec(-1, 0).norm().as_tuple())
    assert physics.Vec.from_bearing(physics.Vec(0, -1).bearing()).as_tuple() == pytest.approx(physics.Vec(0, -1).norm().as_tuple())


def test_bearing_to():
    assert physics.Vec(0, 0).bearing_to((1, 0)) == math.pi / 2
    assert physics.Vec(0, 0).bearing_to((-1, 0)) == -math.pi / 2
    assert physics.Vec(0, 0).bearing_to((0, 1)) == 0
    assert physics.Vec(0, 0).bearing_to((0, -1)) == math.pi


def test_vec_str():
    assert str(physics.Vec(1, 3)) == "(1, 3)"
    assert repr(physics.Vec(1, 3)) == "Vec(1, 3)"


def test_python_conv():
    v = physics.Vec(1.1, 2.1)

    assert len(v) == 2
    assert v[0] == v[-2] == 1.1
    assert v[1] == v[-1] == 2.1

    with pytest.raises(IndexError):
        _ = v[2]

    with pytest.raises(IndexError):
        _ = v[-3]

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


def test_rotate():
    assert physics.Vec(1, 0).rotate(0).as_tuple() == pytest.approx((1, 0))
    assert physics.Vec(1, 0).rotate(math.pi).as_tuple() == pytest.approx((-1, 0))
    assert physics.Vec(0, 1).rotate(math.pi).as_tuple() == pytest.approx((0, -1))
    assert physics.Vec(2, 1).rotate(math.pi).as_tuple() == pytest.approx((-2, -1))
    assert physics.Vec(2, 1).rotate(math.pi / 2).as_tuple() == pytest.approx((1, -2))
    assert physics.Vec(2, 1).rotate(-math.pi / 2).as_tuple() == pytest.approx((-1, 2))
    assert physics.Vec(0, 1).rotate(math.pi / 4).as_tuple() == pytest.approx((2 ** -0.5, 2 ** -0.5))
    assert physics.Vec(1, 1).rotate(math.pi / 4).as_tuple() == pytest.approx((2 ** 0.5, 0))
