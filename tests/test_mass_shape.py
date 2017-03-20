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
