import pytest


def test_distance_between_square_square(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, p2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, 0)
    assert db.b_point.as_tuple() == (9, 0)

    b1.teleport((1, 0))
    b2.teleport((4, 0))

    db = physics.distance_between(p1, p2)

    assert db.distance == pytest.approx(1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (3, 0)

    b2.teleport((3, 0))

    db = physics.distance_between(p1, p2)

    assert db.distance == pytest.approx(0)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (2, 0)

    b2.teleport((2, 0))

    db = physics.distance_between(p1, p2)
    assert db.distance == pytest.approx(-1)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (2, 0)
    assert db.b_point.as_tuple() == (1, 0)

    b2.teleport((1, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-2)

    b2.teleport((1, 1))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-1)

    b1.teleport((0, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-1)

    b2.teleport((2, 2))

    assert physics.distance_between(p1, p2).distance == pytest.approx(0)

    b2.teleport((3, 3))

    assert physics.distance_between(p1, p2).distance == pytest.approx(2 ** 0.5)


def test_distance_between_square_triangle(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(-1, -1), (1, -1), (0, 1)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, p2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, -1)
    assert db.b_point.as_tuple() == (9, -1)

    db = physics.distance_between(p2, p1)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((-1, 0))
    assert db.a_point.as_tuple() == (9, -1)
    assert db.b_point.as_tuple() == (1, -1)

    p1.position = (1, 0)
    b2.teleport((4, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(1)

    b2.teleport((3, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(0)

    b2.teleport((2, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-2 * 5 ** -0.5)

    b2.teleport((1, 0))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-4 * 5 ** -0.5)

    b2.teleport((1, 1))

    assert physics.distance_between(p1, p2).distance == pytest.approx(-1)

    p1.position = (0, 0)

    assert physics.distance_between(p1, p2).distance == pytest.approx(-1)

    b2.teleport((2, 2))

    assert physics.distance_between(p1, p2).distance == pytest.approx(0)


def test_distance_between_triangle_triangle(physics):
    b1 = physics.Body(position=(0, 0))
    p1 = physics.Polygon(points=[(0, 0), (1, 1), (1, 0)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0))
    p2 = physics.Polygon(points=[(1.5, 1), (2, 0), (-1, 0.5)])
    b2.add_shape(p2)

    db = physics.distance_between(p1, p2)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((1, 0))
    assert db.a_point.as_tuple() == (1, 0.5)
    assert db.b_point.as_tuple() == (9, 0.5)

    db = physics.distance_between(p2, p1)

    assert db.distance == pytest.approx(8)
    assert db.normal.as_tuple() == pytest.approx((-1, 0))
    assert db.a_point.as_tuple() == (9, 0.5)
    assert db.b_point.as_tuple() == (1, 0.5)


def test_square_square_horizontal(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(10, 0), velocity=(-6, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((1, 0))
    assert coll.touch_point.as_tuple() == pytest.approx((3, 0))
    assert coll.a is b1
    assert coll.b is b2


def test_square_square_vertical(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1.5, False)
    assert coll.time == pytest.approx(1.0)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((0, 3))
    assert coll.a is b1
    assert coll.b is b2


def test_square_square_vertical_near_hit(physics):
    b1 = physics.Body(position=(1.99999, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1.5, False)
    assert coll.time == pytest.approx(1)
    assert coll.normal.as_tuple() == pytest.approx((0, 1))
    assert coll.touch_point.as_tuple() == pytest.approx((1.99999 / 2, 3))
    assert coll.a is b1
    assert coll.b is b2


def test_square_square_vertical_near_miss(physics):
    b1 = physics.Body(position=(2.001, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1, False)
    assert coll.time == -1


def test_square_square_no_collision_opposite_dir(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(0, 6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1, False)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_square_square_no_collision_parallel(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 10), velocity=(2, 0))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1, False)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_square_square_out_of_time(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 2))
    p1 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b1.add_shape(p1)

    b2 = physics.Body(position=(0, 100), velocity=(0, -6))
    p2 = physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)])
    b2.add_shape(p2)

    coll = physics.collide_shapes(p1, p2, 1, False)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None
