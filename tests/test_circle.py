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
