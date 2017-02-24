import physics


def test_circle_circle_horizontal():
    b1 = physics.Body()
    b1.position = 0, 0
    b1.velocity = 2, 0
    c1 = physics.Circle()
    c1.radius = 1
    b1.add_shape(c1)

    b2 = physics.Body()
    b2.position = 10, 0
    b2.velocity = -6, 0
    c2 = physics.Circle()
    c2.radius = 1
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1)
    assert coll.time == 1.0
    assert coll.normal.as_tuple() == (1, 0)
    assert coll.touch_point.as_tuple() == (3, 0)
    assert coll.a is b1
    assert coll.b is b2


def test_circle_circle_vertical():
    b1 = physics.Body()
    b1.position = 0, 0
    b1.velocity = 0, 2
    c1 = physics.Circle()
    c1.radius = 1
    b1.add_shape(c1)

    b2 = physics.Body()
    b2.position = 0, 10
    b2.velocity = 0, -6
    c2 = physics.Circle()
    c2.radius = 1
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1)
    assert coll.time == 1.0
    assert coll.normal.as_tuple() == (0, 1)
    assert coll.touch_point.as_tuple() == (0, 3)
    assert coll.a is b1
    assert coll.b is b2


def test_circle_circle_no_collision_opposite_dir():
    b1 = physics.Body()
    b1.position = 0, 0
    b1.velocity = 0, 2
    c1 = physics.Circle()
    c1.radius = 1
    b1.add_shape(c1)

    b2 = physics.Body()
    b2.position = 0, 10
    b2.velocity = 0, 6
    c2 = physics.Circle()
    c2.radius = 1
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_circle_no_collision_parallel():
    b1 = physics.Body()
    b1.position = 0, 0
    b1.velocity = 2, 0
    c1 = physics.Circle()
    c1.radius = 1
    b1.add_shape(c1)

    b2 = physics.Body()
    b2.position = 0, 10
    b2.velocity = 2, 0
    c2 = physics.Circle()
    c2.radius = 1
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None


def test_circle_circle_out_of_time():
    b1 = physics.Body()
    b1.position = 0, 0
    b1.velocity = 0, 2
    c1 = physics.Circle()
    c1.radius = 1
    b1.add_shape(c1)

    b2 = physics.Body()
    b2.position = 0, 100
    b2.velocity = 0, -6
    c2 = physics.Circle()
    c2.radius = 1
    b2.add_shape(c2)

    coll = physics.collide_circle_circle(c1, c2, 1)
    assert coll.time == -1.0
    assert coll.a is None
    assert coll.b is None
