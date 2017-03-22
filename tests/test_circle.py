def test_clone(physics):
    c = physics.Circle(radius=5, mass=10)

    assert c.mass == 10
    assert c.radius == 5

    c.mass = 50

    assert c.mass == 50

    c2 = c.clone()

    assert c2.mass == 50
    assert c2.radius == 5
    assert type(c2) is type(c)

    c2.mass = 55
    c2.radius = 10

    assert c.mass == 50
    assert c2.mass == 55
    assert c.radius == 5
    assert c2.radius == 10


def test_aabb(physics):
    c = physics.Circle(radius=5, mass=10)

    assert c.can_collide()
    assert c.aabb(0).as_tuple() == (-5, 5, -5, 5)
    assert c.aabb(1).as_tuple() == (-5, 5, -5, 5)

    c = physics.Circle(radius=100, mass=10)

    assert c.aabb(0).as_tuple() == (-100, 100, -100, 100)
    assert c.aabb(1).as_tuple() == (-100, 100, -100, 100)


def test_moi(physics):
    c = physics.Circle(radius=5, mass=10)

    assert c.moment_of_inertia == 125

    c = physics.Circle(radius=10, mass=10)

    assert c.moment_of_inertia == 500

    c = physics.Circle(radius=10, mass=5)

    assert c.moment_of_inertia == 250
