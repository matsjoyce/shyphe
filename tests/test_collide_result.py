import physics
import random
import math
import pytest


def dummy_shape(mass):
    return physics.MassShape(mass=mass)


def test_complex_collision():
    a = physics.Body(position=(0, 0), velocity=(30, 40))
    a.add_shape(dummy_shape(10))
    b = physics.Body(position=(2, 0), velocity=(-30, 0))
    b.add_shape(dummy_shape(10))

    col = physics.CollisionTimeResult(a, b, 0, (0, 0), (b.position - a.position).norm(), True)

    cr = physics.collision_result(col, physics.CollisionParameters(1, 10000, 1))

    assert cr.impulse.as_tuple() == (-600, 0)
    assert cr.closing_velocity.as_tuple() == (-60, 0)


def test_same_direction_vertical():
    a = physics.Body(position=(0, 2), velocity=(0, -30))
    a.add_shape(dummy_shape(10))
    b = physics.Body(position=(0, 0), velocity=(0, -20))
    b.add_shape(dummy_shape(10))

    col = physics.CollisionTimeResult(a, b, 0, (0, 0), (b.position - a.position).norm(), True)

    cr = physics.collision_result(col, physics.CollisionParameters(1, 10000, 1))

    assert cr.impulse.as_tuple() == (0, 100)
    assert cr.closing_velocity.as_tuple() == (0, 10)


def test_same_direction_horizontal():
    a = physics.Body(position=(0, 0), velocity=(-20, 0))
    a.add_shape(dummy_shape(10))
    b = physics.Body(position=(2, 0), velocity=(-30, 0))
    b.add_shape(dummy_shape(10))

    col = physics.CollisionTimeResult(a, b, 0, (0, 0), (b.position - a.position).norm(), True)

    cr = physics.collision_result(col, physics.CollisionParameters(1, 10000, 1))

    assert cr.impulse.as_tuple() == (-100, 0)
    assert cr.closing_velocity.as_tuple() == (-10, 0)


def test_transition_reduction():
    a = physics.Body(position=(0, 2), velocity=(0, -300))
    a.add_shape(dummy_shape(10))
    b = physics.Body(position=(0, 0), velocity=(0, -200))
    b.add_shape(dummy_shape(10))

    col = physics.CollisionTimeResult(a, b, 0, (0, 0), (b.position - a.position).norm(), True)

    cr = physics.collision_result(col, physics.CollisionParameters(1, 500, 0.1))

    assert cr.impulse.as_tuple() == (0, 550)
    assert cr.closing_velocity.as_tuple() == (0, 100)


@pytest.mark.parametrize("i", range(20))
def test_collision_fuzz(i):
    a = physics.Body(position=(random.randrange(0, 50), random.randrange(0, 50)))
    a.velocity = 20 * physics.Vec.from_bearing((random.random() - 0.5) * math.pi)
    a.add_shape(dummy_shape(1))
    b = physics.Body(position=(random.randrange(0, 50), random.randrange(0, 50)))
    b.velocity = (a.position - b.position).norm() * 30
    b.add_shape(dummy_shape(1))

    d1 = (a.position - b.position).abs()
    if d1 == 0:
        return

    col = physics.CollisionTimeResult(a, b, 0, (0, 0), (b.position - a.position).norm(), True)

    cr = physics.collision_result(col, physics.CollisionParameters(1, 10000, 1))

    a.apply_impulse(cr.impulse, col.touch_point - a.position)
    b.apply_impulse(-cr.impulse, col.touch_point - b.position)

    a.update_position(1)
    b.update_position(1)

    assert d1 <= (a.position - b.position).abs()
