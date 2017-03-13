import math
import random
import pytest


def test_to_deg(physics):
    assert physics.to_deg(0) == 0
    assert physics.to_deg(math.pi / 2) == 90
    assert physics.to_deg(math.pi) == 180
    assert physics.to_deg(-math.pi / 2) == 270

    for i in range(0, 1000, 4):
        assert physics.to_deg(i) == math.degrees(i) % 360


def test_to_rad(physics):
    assert physics.to_rad(0) == 0
    assert physics.to_rad(90) == math.pi / 2
    assert physics.to_rad(180) == math.pi or physics.to_rad(270) == -math.pi
    assert physics.to_rad(270) == -math.pi / 2

    for i in range(0, 1000, 4):
        if i % 360 == 180:
            assert physics.to_rad(i) == math.pi or physics.to_rad(i) == -math.pi
        else:
            x = math.radians(i) % (2 * math.pi)
            x = min(x, x - 2 * math.pi, key=abs)
            assert physics.to_rad(i) == pytest.approx(x)


def test_to_rad_fuzz(physics):
    for i in range(1000):
        x = (2 * random.random() - 1) * math.pi
        assert pytest.approx(x) == physics.to_rad(physics.to_deg(x))


def test_to_deg_fuzz(physics):
    for i in range(1000):
        x = random.random() * 360
        assert pytest.approx(x) == physics.to_deg(physics.to_rad(x))
