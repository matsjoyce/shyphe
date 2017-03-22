def test_clone(physics):
    m = physics.MassShape(mass=45)

    assert m.mass == 45

    m.mass = 50

    assert m.mass == 50

    m2 = m.clone()

    assert m2.mass == 50
    assert type(m2) is type(m)

    m2.mass = 55

    assert m.mass == 50
    assert m2.mass == 55


def test_aabb(physics):
    m = physics.MassShape(mass=45)

    assert not m.can_collide()
    assert m.aabb(0).as_tuple() == (0, 0, 0, 0)
    assert m.aabb(1).as_tuple() == (0, 0, 0, 0)


def test_moi(physics):
    m = physics.MassShape(mass=45)

    assert m.moment_of_inertia == 1
