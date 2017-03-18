import pytest


def test_immediate_collide_simple(physics):
    b1 = physics.Body(position=(0, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    assert not physics.immediate_collide_circle_polygon(c, p)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert not physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((2.9, 0))

    assert physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0, 3))

    assert not physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0, 1.9))

    assert physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0.5, 0.5))

    assert physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0, 0))

    assert physics.immediate_collide_circle_polygon(c, p)


def test_immediate_collide_rotated(physics):
    b1 = physics.Body(position=(0, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0))
    p = physics.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)])
    b2.add_shape(p)

    assert not physics.immediate_collide_circle_polygon(c, p)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert not physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((2.9, 0))

    assert physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0, 3))

    assert not physics.immediate_collide_circle_polygon(c, p)

    x = (1 + 2 ** -0.5) * 2 ** -0.5

    b1.teleport((0, 0))
    b2.teleport((x - 0.0001, x - 0.0001))

    assert physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((x + 0.0001, x + 0.0001))

    assert not physics.immediate_collide_circle_polygon(c, p)

    b2.teleport((0, 0))

    assert physics.immediate_collide_circle_polygon(c, p)


def test_circle_polygon_horizontal(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0), velocity=(-6, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))
    assert coll.a is b1
    assert coll.b is b2
    assert coll.entering

    coll = physics.collide_circle_polygon(c, p, 2, False)
    assert coll.time == pytest.approx(1.5)
    assert coll.normal.as_tuple() == pytest.approx((-1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((2, 0))
    assert coll.a is b1
    assert coll.b is b2
    assert not coll.entering


def test_circle_polygon_vertical(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))
    assert coll.a is b1
    assert coll.b is b2
    assert coll.entering

    coll = physics.collide_circle_polygon(c, p, 2, False)
    assert coll.time == pytest.approx(1.5)
    assert coll.normal.as_tuple() == pytest.approx((0, -1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 2))
    assert coll.a is b1
    assert coll.b is b2
    assert not coll.entering


def test_circle_polygon_vertical_near_hit(physics):
    b1 = physics.Body(position=(1.999, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == pytest.approx(1.25)


def test_circle_polygon_vertical_near_miss(physics):
    b1 = physics.Body(position=(2.001, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == -1


def test_circle_polygon_no_collision_opposite_dir(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, 6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_polygon_no_collision_parallel(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(2, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_polygon_out_of_time(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 100), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_circle_polygon(c, p, 1, True)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None
