import pytest


def test_single_bounce(physics):
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(8, 0), velocity=(-6, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1)
    c.add_body(b1)
    c.add_body(b2)
    c.begin_frame()

    assert c.has_next_collision()
    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == pytest.approx(0.75)
    assert col1.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col1.impulse.as_tuple() == pytest.approx((-8, 0))
    assert col2.impulse.as_tuple() == pytest.approx((8, 0))
    assert col1.closing_velocity.as_tuple() == pytest.approx((-8, 0))
    assert col2.closing_velocity.as_tuple() == pytest.approx((8, 0))
    assert col1.body.position.as_tuple() == pytest.approx((1.5, 0))
    assert col2.body.position.as_tuple() == pytest.approx((3.5, 0))

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-6, 0)
    assert b2.velocity.as_tuple() == (2, 0)

    c.finished_collision(ctr, True)

    assert not c.has_next_collision()

    c.end_frame()

    assert b1.position.as_tuple() == (0, 0)
    assert b2.position.as_tuple() == (4, 0)

    c.begin_frame()

    assert not c.has_next_collision()

    c.end_frame()

    assert b1.position.as_tuple() == (-6, 0)
    assert b2.position.as_tuple() == (6, 0)


def test_rotating_hit(physics):
    b1 = physics.Body(position=(0, 0), angular_velocity=1, angle=-1)
    b1.add_shape(physics.Polygon(points=[(-10, 1), (-10, -1), (10, -1), (10, 1)], mass=1, position=(10, 0)))

    b2 = physics.Body(position=(8, -2))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.begin_frame()

    assert c.has_next_collision()
    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == pytest.approx(1)
    assert col1.touch_point.as_tuple() == pytest.approx((8, -1))
    assert col2.touch_point.as_tuple() == pytest.approx((0, 1), abs=1e-8)
    assert col1.closing_velocity.as_tuple() == pytest.approx((0, 8), abs=1e-8)
    assert col2.closing_velocity.as_tuple() == pytest.approx((0, -8), abs=1e-8)
    assert col1.body.position.as_tuple() == pytest.approx((0, 0))
    assert col2.body.position.as_tuple() == pytest.approx((8, -2))

    col1.apply_impulse()
    col2.apply_impulse()
    c.finished_collision(ctr, True)

    assert not c.has_next_collision()


def test_ignore_rotating_hit(physics):
    b1 = physics.Body(position=(0, 0), angular_velocity=1, angle=-1)
    b1.add_shape(physics.Polygon(points=[(-10, 1), (-10, -1), (10, -1), (10, 1)], mass=1, position=(10, 0)))

    b2 = physics.Body(position=(8, -2))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.begin_frame()

    assert c.has_next_collision()
    ctr = c.next_collision()
    c.finished_collision(ctr, False)

    assert not c.has_next_collision()


def test_miss_with_aabb_hit(physics):
    b1 = physics.Body(position=(0, 0), velocity=(0, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(-4, -0.9), velocity=(4, 4))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(2)
    c.add_body(b1)
    c.add_body(b2)
    c.begin_frame()

    assert not c.has_next_collision()

    with pytest.raises(RuntimeError):
        c.next_collision()


def test_double_bounce(physics):
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(8, 0), velocity=(0, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(0.75)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    assert c.has_next_collision()

    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == pytest.approx(0.25)
    assert col1.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-4, 0)
    assert b2.velocity.as_tuple() == (4, 0)

    c.finished_collision(ctr, True)

    assert not c.has_next_collision()

    c.end_frame()
    c.begin_frame()

    assert c.has_next_collision()

    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b3 or cola.body is b2
    assert colb.body is b3 or colb.body is b2

    if cola.body is b3:
        col3, col2 = cola, colb
    else:
        col3, col2 = colb, cola

    assert col3.time == col2.time == pytest.approx(1)
    assert col3.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col3.impulse.as_tuple() == (4, 0)
    assert col2.impulse.as_tuple() == (-4, 0)
    assert col3.closing_velocity.as_tuple() == (4, 0)
    assert col2.closing_velocity.as_tuple() == (-4, 0)

    b3.apply_impulse(col3.impulse, col3.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b3.velocity.as_tuple() == (4, 0)
    assert b2.velocity.as_tuple() == (0, 0)

    c.finished_collision(ctr, True)

    assert not c.has_next_collision()


def test_double_bounce_poly(physics):
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)], mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b2.add_shape(physics.Polygon(points=[(0, 1), (1, 0), (0, -1), (-1, 0)], mass=1))

    b3 = physics.Body(position=(8, 0), velocity=(0, 0))
    b3.add_shape(physics.Polygon(points=[(-1, -1), (-1, 1), (1, 1), (1, -1)], mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    assert c.has_next_collision()

    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == pytest.approx(0.25)
    assert col1.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    col1.apply_impulse()
    col2.apply_impulse()

    assert b1.velocity.as_tuple() == (-4, 0)
    assert b2.velocity.as_tuple() == (4, 0)

    c.finished_collision(ctr, True)

    assert c.has_next_collision()

    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b3 or cola.body is b2
    assert colb.body is b3 or colb.body is b2

    if cola.body is b3:
        col3, col2 = cola, colb
    else:
        col3, col2 = colb, cola

    assert col3.time == col2.time == pytest.approx(1)
    assert col3.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col3.impulse.as_tuple() == (4, 0)
    assert col2.impulse.as_tuple() == (-4, 0)
    assert col3.closing_velocity.as_tuple() == (4, 0)
    assert col2.closing_velocity.as_tuple() == (-4, 0)

    col2.apply_impulse()
    col3.apply_impulse()

    assert b3.velocity.as_tuple() == (4, 0)
    assert b2.velocity.as_tuple() == (0, 0)

    c.finished_collision(ctr, True)

    assert not c.has_next_collision()


def test_simultaneous(physics):
    b1 = physics.Body(position=(-4, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(0, 0), velocity=(0, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    b4 = physics.Body(position=(0, -4), velocity=(0, 4))
    b4.add_shape(physics.Circle(radius=1, mass=1))

    b5 = physics.Body(position=(0, 4), velocity=(0, -4))
    b5.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.add_body(b4)
    c.add_body(b5)
    c.begin_frame()

    collisions = 0
    while c.has_next_collision():
        ctr = c.next_collision()
        cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))
        cola.body.apply_impulse(cola.impulse, cola.touch_point)
        colb.body.apply_impulse(colb.impulse, colb.touch_point)
        c.finished_collision(ctr, True)
        collisions += 1

    c.end_frame()

    assert collisions >= 6
    assert b1.velocity.x < b2.velocity.x < b3.velocity.x
    assert b4.velocity.y < b2.velocity.y < b5.velocity.y


def test_triple(physics):
    b1 = physics.Body(position=(-4, 0), velocity=(4.1, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(0, 0), velocity=(0, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    collisions = 0
    while c.has_next_collision():
        ctr = c.next_collision()
        cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))
        cola.body.apply_impulse(cola.impulse, cola.touch_point)
        colb.body.apply_impulse(colb.impulse, colb.touch_point)
        c.finished_collision(ctr, True)
        collisions += 1

    c.end_frame()

    assert b1.velocity.x < b2.velocity.x < b3.velocity.x
    assert collisions >= 2


def test_triple_inelastic(physics):
    b1 = physics.Body(position=(-3, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(0, 0), velocity=(0, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(3, 0), velocity=(-4, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(1.5)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    collisions = 0
    while c.has_next_collision():
        ctr = c.next_collision()
        cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(0.5))
        cola.body.apply_impulse(cola.impulse, cola.touch_point)
        colb.body.apply_impulse(colb.impulse, colb.touch_point)
        c.finished_collision(ctr, True)
        collisions += 1

    c.end_frame()

    assert b1.velocity.x < b2.velocity.x < b3.velocity.x
    assert collisions > 2


def test_single_bounce_die(physics):
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(8, 0), velocity=(0, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.World(2)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    assert c.has_next_collision()

    ctr = c.next_collision()
    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == pytest.approx(0.25)
    assert col1.touch_point.as_tuple() == pytest.approx((1, 0))
    assert col2.touch_point.as_tuple() == pytest.approx((-1, 0))
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-4, 0)
    assert b2.velocity.as_tuple() == (4, 0)

    c.remove_body(b2)
    c.finished_collision(ctr, True)

    assert not c.has_next_collision()


def test_ignored(physics):
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(0, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(10, 0), velocity=(0, 0))
    b3.add_shape(physics.Circle(radius=1, mass=10))

    c = physics.World(6)
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.begin_frame()

    assert c.has_next_collision()

    ctr = c.next_collision()

    assert (ctr.a, ctr.b) == (b1, b2) or (ctr.b, ctr.a) == (b1, b2)

    c.finished_collision(ctr, False)

    assert c.has_next_collision()

    ctr = c.next_collision()

    assert (ctr.a, ctr.b) == (b1, b3) or (ctr.b, ctr.a) == (b1, b3)

    cola, colb = c.calculate_collision(ctr, physics.CollisionParameters(1))
    cola.body.apply_impulse(cola.impulse, cola.touch_point)
    colb.body.apply_impulse(colb.impulse, colb.touch_point)

    assert b1.velocity.x < -1

    c.finished_collision(ctr, True)

    assert c.has_next_collision()

    ctr = c.next_collision()

    assert (ctr.a, ctr.b) == (b1, b2) or (ctr.b, ctr.a) == (b1, b2)
