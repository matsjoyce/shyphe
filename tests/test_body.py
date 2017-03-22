import pytest


def test_add_shape(physics):
    b = physics.Body()
    b.add_shape(physics.MassShape(mass=10))

    assert b.mass == 10


def test_remove_shape(physics):
    b = physics.Body()
    c = physics.MassShape(mass=10)
    b.add_shape(c)
    b.remove_shape(c)

    assert b.mass == 0


def test_broken_add_remove(physics):
    b = physics.Body()
    b2 = physics.Body()
    c = physics.MassShape(mass=10)
    b.add_shape(c)

    with pytest.raises(RuntimeError):
        b2.add_shape(c)

    with pytest.raises(RuntimeError):
        b2.remove_shape(c)

    b.remove_shape(c)

    with pytest.raises(RuntimeError):
        b.remove_shape(c)


def test_body_distance(physics):
    b1 = physics.Body(position=(0, 0))
    b1.add_shape(physics.Circle(radius=1, position=(1, 0)))
    b1.add_shape(physics.Circle(radius=1, position=(-1, 0)))
    b1.add_shape(physics.MassShape(position=(5, 0)))

    b2 = physics.Body(position=(10, 0))
    b2.add_shape(physics.Circle(radius=1, position=(0, 1)))
    b2.add_shape(physics.Circle(radius=1, position=(-1, 0)))
    b2.add_shape(physics.MassShape(position=(-5, 0)))

    assert b1.distance_between(b2) == 6


def test_body_collide(physics):
    b1 = physics.Body(position=(0, 0), velocity=(1, 0))
    b1.add_shape(physics.Circle(radius=1, position=(1, 0)))
    b1.add_shape(physics.Circle(radius=1, position=(-1, 0)))
    b1.add_shape(physics.MassShape(position=(5, 0)))

    b2 = physics.Body(position=(10, 0), velocity=(-5, 0))
    b2.add_shape(physics.Circle(radius=1, position=(0, 1)))
    b2.add_shape(physics.Circle(radius=1, position=(-1, 0)))
    b2.add_shape(physics.MassShape(position=(-5, 0)))

    colr = b1.collide(b2, 2, False)

    assert colr.time == pytest.approx(1.0)


def test_local_linear_acceleration(physics):
    b = physics.Body()
    c = physics.MassShape(mass=1)
    b.add_shape(c)
    b.apply_local_force((1, 0), (0, 0))

    b.update_velocity(1)

    assert b.velocity == (1, 0)

    b.update_velocity(1)

    assert b.velocity == (2, 0)

    b.update_velocity(1)

    assert b.velocity == (3, 0)

    b.clear_local_forces()

    b.update_velocity(1)

    assert b.velocity == (3, 0)


def test_global_acceleration(physics):
    b = physics.Body()
    c = physics.MassShape(mass=1)
    b.add_shape(c)
    b.apply_global_force((1, 0), (0, 0))

    b.update_velocity(1)

    assert b.velocity == (1, 0)

    b.update_velocity(1)

    assert b.velocity == (2, 0)

    b.update_velocity(1)

    assert b.velocity == (3, 0)

    b.clear_global_forces()

    b.update_velocity(1)

    assert b.velocity == (3, 0)


def test_aabb(physics):
    b = physics.Body()
    c = physics.Circle(radius=1, mass=1)
    b.add_shape(c)

    assert b.aabb(0).as_tuple() == (-1, 1, -1, 1)
    assert b.aabb(1).as_tuple() == (-1, 1, -1, 1)
    assert b.aabb(0.01).as_tuple() == (-1, 1, -1, 1)

    c.position = (1, 0)

    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (0, 2, -1, 1)

    b.apply_impulse((1, 0), (0, 0))

    assert b.velocity.as_tuple() == (1, 0)

    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (0, 3, -1, 1)

    b.apply_impulse((-1, -1), (0, 0))
    b.apply_impulse((0, 1), (3 / 2 * physics.to_rad(45), 0))

    assert b.angular_velocity == -physics.to_rad(45)
    assert b.velocity.as_tuple() == (0, 0)
    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    assert b.aabb(1).as_tuple() == (2 ** -0.5 - 1, 2, -1, 2 ** -0.5 + 1)
    assert b.aabb(2).as_tuple() == pytest.approx((-1, 2, -1, 2))
    assert b.aabb(3).as_tuple() == (-2 ** -0.5 - 1, 2, -1, 2)
    assert b.aabb(4).as_tuple() == pytest.approx((-2, 2, -1, 2))

    b.apply_impulse((1, 0), (0, 0))

    assert b.angular_velocity == -physics.to_rad(45)
    assert b.velocity.as_tuple() == (1, 0)
    assert b.aabb(0).as_tuple() == (0, 2, -1, 1)
    #assert b.aabb(1).as_tuple() == (2 ** -0.5 - 1, 2, -1, 2 ** -0.5 + 1)
    #assert b.aabb(2).as_tuple() == pytest.approx((-1, 3, -1, 2))
    #assert b.aabb(3).as_tuple() == (-2 ** -0.5 - 1, 2, -1, 2)
    #assert b.aabb(4).as_tuple() == pytest.approx((-2, 4, -1, 2))
