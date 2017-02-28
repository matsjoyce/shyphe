import physics
import pytest


def test_clone():
    s = physics.ActiveRadar(power=50, sensitivity=2)
    s2 = s.clone()

    assert s.power == s2.power
    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)

    s = physics.PassiveRadar(sensitivity=3)
    s2 = s.clone()

    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)

    s = physics.PassiveThermal(sensitivity=4)
    s2 = s.clone()

    assert s.sensitivity == s2.sensitivity
    assert s.max_range == s2.max_range
    assert type(s) is type(s2)


def test_perf():
    s = physics.ActiveRadar(power=50, sensitivity=2)

    assert s.max_range == 1250

    s.perf = 0.5

    assert s.max_range == 1250 / 2

    s = physics.PassiveRadar(sensitivity=3)

    assert s.max_range == 50 / 3

    s.perf = 0.5

    assert s.max_range == 50 / 6

    s = physics.PassiveThermal(sensitivity=3)

    assert s.max_range == 2500 / 3

    s.perf = 0.5

    assert s.max_range == 2500 / 6
