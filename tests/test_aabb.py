def test_corners(physics):
    aabb = physics.AABB(-1, 2, 0, 5)

    assert aabb.center.as_tuple() == (0.5, 2.5)
    assert aabb.topleft.as_tuple() == (-1, 5)
    assert aabb.topright.as_tuple() == (2, 5)
    assert aabb.bottomleft.as_tuple() == (-1, 0)
    assert aabb.bottomright.as_tuple() == (2, 0)


def test_construct(physics):
    x = physics.AABB(-1, 2, 0, 5)
    y = physics.AABB((0.5, 2.5), 3, 5)
    z = physics.AABB((-1, 5), (2, 0))

    assert x.as_tuple() == y.as_tuple() == z.as_tuple()


def test_aabb_str(physics):
    assert str(physics.AABB(-1, 2, 0, 5)) == "AABB(-1 - 2, 0 - 5)"
    assert repr(physics.AABB(-1, 2, 0, 5)) == "AABB(-1, 2, 0, 5)"
