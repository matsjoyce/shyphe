import pytest


def test_clone(physics):
    p = physics.Polygon(points=[(0, 0), (1, 1), (1, 0)], mass=10)

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


def test_bad_shaped(physics):
    # concave
    with pytest.raises(RuntimeError):
        physics.Polygon(points=[(0, 1), (1, 1), (1, 0), (0.8, 0.8)], mass=10)

    # not enough points
    with pytest.raises(RuntimeError):
        physics.Polygon(points=[(0, 0), (1, 1)])


def test_normalization(physics):
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    p2 = physics.Polygon(points=[(1, -1), (1, 1), (-1, 1), (-1, -1)])

    assert list(p1.points) == list(p2.points) == [physics.Vec(-1, -1), physics.Vec(-1, 1), physics.Vec(1, 1), physics.Vec(1, -1)]


def test_degenerate(physics):
    p1 = physics.Polygon(points=[(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)])

    assert list(p1.points) == [physics.Vec(-1, -1), physics.Vec(-1, 1), physics.Vec(1, 1), physics.Vec(1, -1)]

