import physics
import math
import pytest


@pytest.mark.xfail
def test_to_deg():
    for i in range(0, 1000, 4):
        assert physics.to_deg(i) == math.degrees(i)


@pytest.mark.xfail
def test_to_rad():
    for i in range(0, 1000, 4):
        assert physics.to_rad(i) == math.radians(i)
