import pytest


def test_circle_circle_horizontal(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c1 = physics.Circle(radius=1)
    b1.add_shape(c1)

    b2 = physics.Body(position=(10, 0), velocity=(-6, 0))
    c2 = physics.Circle(radius=1)
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1, True)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))
    assert coll.a is b1
    assert coll.b is b2
    assert coll.entering


def test_circle_circle_vertical(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c1 = physics.Circle(radius=1)
    b1.add_shape(c1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    c2 = physics.Circle(radius=1)
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1, True)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))
    assert coll.a is b1
    assert coll.b is b2
    assert coll.entering


def test_circle_circle_no_collision_opposite_dir(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c1 = physics.Circle(radius=1)
    b1.add_shape(c1)

    b2 = physics.Body(position=(0, 10), velocity=(0, 6))
    c2 = physics.Circle(radius=1)
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_circle_no_collision_parallel(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c1 = physics.Circle(radius=1)
    b1.add_shape(c1)

    b2 = physics.Body(position=(0, 10), velocity=(2, 0))
    c2 = physics.Circle(radius=1)
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_circle_out_of_time(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c1 = physics.Circle(radius=1)
    b1.add_shape(c1)

    b2 = physics.Body(position=(0, 100), velocity=(0, -6))
    c2 = physics.Circle(radius=1)
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None
