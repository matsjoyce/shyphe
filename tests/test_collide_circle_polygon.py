import pytest


def test_distance_between_simple(physics):
    b1 = physics.Body(position=(0, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b2.teleport((0, 3))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(1)

    b2.teleport((0, 1.9))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b2.teleport((0.5, 0.5))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-1.5)

    b2.teleport((0, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-1)


def test_distance_between_simple2(physics):
    b1 = physics.Body(position=(0, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0))
    p = physics.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)])
    b2.add_shape(p)

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    c.position = (1, 0)
    b2.teleport((4, 0))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    c.position = (0, 0)
    b2.teleport((0, 3))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    x = (1 + 2 ** -0.5) * 2 ** -0.5

    b2.teleport((x - 0.0001, x - 0.0001))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-(0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((x + 0.0001, x + 0.0001))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx((0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((0, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-1 - 2 ** -0.5)


def test_distance_between_rotated(physics):
    b1 = physics.Body(position=(0, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0))
    p = physics.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)])
    b2.add_shape(p)

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(8)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    b2.teleport((2.9, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-0.1)

    b1.teleport((0, 0))
    b2.teleport((0, 3))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx(1)

    x = (1 + 2 ** -0.5) * 2 ** -0.5

    b2.teleport((x - 0.0001, x - 0.0001))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-(0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((x + 0.0001, x + 0.0001))

    assert physics.distance_between(p, b2, c, b1).distance == pytest.approx((0.0001 ** 2 * 2) ** 0.5)

    b2.teleport((0, 0))

    assert physics.distance_between(c, b1, p, b2).distance == pytest.approx(-1 - 2 ** -0.5)


def test_circle_polygon_horizontal(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(10, 0), velocity=(-6, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))


def test_circle_polygon_vertical(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))


def test_circle_polygon_vertical_near_hit(physics):
    b1 = physics.Body(position=(2, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1.5, False)
    assert coll.time == pytest.approx(1.125, 1e-5)


def test_circle_polygon_vertical_near_miss(physics):
    b1 = physics.Body(position=(2.001, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1


def test_circle_polygon_no_collision_opposite_dir(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(0, 6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0


def test_circle_polygon_no_collision_parallel(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 10), velocity=(2, 0))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0


def test_circle_polygon_out_of_time(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    c = physics.Circle(radius=1)
    b1.add_shape(c)

    b2 = physics.Body(position=(0, 100), velocity=(0, -6))
    p = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p)

    coll = physics.collide_shapes(c, b1, p, b2, 1, False)
    assert coll.time == -1.0
