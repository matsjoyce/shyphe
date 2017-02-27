import physics
import pytest


def test_add_shape():
    b = physics.Body()
    b.add_shape(physics.Circle(mass=10))

    assert b.mass == 10


def test_remove_shape():
    b = physics.Body()
    c = physics.Circle(mass=10)
    b.add_shape(c)
    b.remove_shape(c)

    assert b.mass == 0


def test_broken_add_remove():
    b = physics.Body()
    b2 = physics.Body()
    c = physics.Circle(mass=10)
    b.add_shape(c)

    with pytest.raises(RuntimeError):
        b2.add_shape(c)

    with pytest.raises(RuntimeError):
        b2.remove_shape(c)

    b.remove_shape(c)

    with pytest.raises(RuntimeError):
        b.remove_shape(c)


def test_clone_shape():
    b = physics.Body()
    b2 = physics.Body()
    c = physics.Circle(mass=10)
    b.add_shape(c)
    b2.add_shape(c.clone())

    assert b.mass == b.mass == 10
