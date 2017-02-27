import physics
import pytest


def test_single_bounce():
    b1 = physics.Body(position=(0, 0), velocity=(2, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(8, 0), velocity=(-6, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.Collider()
    c.add_body(b1)
    c.add_body(b2)
    c.reset(1)

    assert c.has_next_collision()
    cola, colb = c.next_collision()

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == 0.75
    assert col1.touch_point.as_tuple() == (1, 0)
    assert col2.touch_point.as_tuple() == (-1, 0)
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-6, 0)
    assert b2.velocity.as_tuple() == (2, 0)

    c.finished_collision()

    assert not c.has_next_collision()
    assert b1.position.as_tuple() == (0, 0)
    assert b2.position.as_tuple() == (4, 0)

    c.reset(1)

    assert not c.has_next_collision()
    assert b1.position.as_tuple() == (-6, 0)
    assert b2.position.as_tuple() == (6, 0)


def test_double_bounce():
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(8, 0), velocity=(0, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.Collider()
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.reset(2)

    assert c.has_next_collision()

    cola, colb = c.next_collision()

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == 0.25
    assert col1.touch_point.as_tuple() == (1, 0)
    assert col2.touch_point.as_tuple() == (-1, 0)
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-4, 0)
    assert b2.velocity.as_tuple() == (4, 0)

    c.finished_collision()

    assert c.has_next_collision()

    cola, colb = c.next_collision()

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b3 or cola.body is b2
    assert colb.body is b3 or colb.body is b2

    if cola.body is b3:
        col3, col2 = cola, colb
    else:
        col3, col2 = colb, cola

    assert col3.time == col2.time == 1
    assert col3.touch_point.as_tuple() == (-1, 0)
    assert col2.touch_point.as_tuple() == (1, 0)
    assert col3.impulse.as_tuple() == (4, 0)
    assert col2.impulse.as_tuple() == (-4, 0)
    assert col3.closing_velocity.as_tuple() == (4, 0)
    assert col2.closing_velocity.as_tuple() == (-4, 0)

    b3.apply_impulse(col3.impulse, col3.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b3.velocity.as_tuple() == (4, 0)
    assert b2.velocity.as_tuple() == (0, 0)

    c.finished_collision()

    assert not c.has_next_collision()


def test_single_bounce_die():
    b1 = physics.Body(position=(0, 0), velocity=(4, 0))
    b1.add_shape(physics.Circle(radius=1, mass=1))

    b2 = physics.Body(position=(4, 0), velocity=(-4, 0))
    b2.add_shape(physics.Circle(radius=1, mass=1))

    b3 = physics.Body(position=(8, 0), velocity=(0, 0))
    b3.add_shape(physics.Circle(radius=1, mass=1))

    c = physics.Collider()
    c.add_body(b1)
    c.add_body(b2)
    c.add_body(b3)
    c.reset(2)

    assert c.has_next_collision()

    cola, colb = c.next_collision()

    assert cola.body is colb.other
    assert colb.body is cola.other

    assert cola.body is b1 or cola.body is b2
    assert colb.body is b1 or colb.body is b2

    if cola.body is b1:
        col1, col2 = cola, colb
    else:
        col1, col2 = colb, cola

    assert col1.time == col2.time == 0.25
    assert col1.touch_point.as_tuple() == (1, 0)
    assert col2.touch_point.as_tuple() == (-1, 0)
    assert col1.impulse.as_tuple() == (-8, 0)
    assert col2.impulse.as_tuple() == (8, 0)
    assert col1.closing_velocity.as_tuple() == (-8, 0)
    assert col2.closing_velocity.as_tuple() == (8, 0)

    b1.apply_impulse(col1.impulse, col1.touch_point)
    b2.apply_impulse(col2.impulse, col2.touch_point)

    assert b1.velocity.as_tuple() == (-4, 0)
    assert b2.velocity.as_tuple() == (4, 0)

    c.remove_body(b2)
    c.finished_collision()

    assert not c.has_next_collision()
