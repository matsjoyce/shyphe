import pytest


def test_clone(physics):
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


def test_body_sensor(physics):
    s = physics.ActiveRadar(power=50, sensitivity=2)
    b = physics.Body()

    assert b.max_sensor_range == 0

    b.add_sensor(s)

    with pytest.raises(RuntimeError):
        b.add_sensor(s)

    assert b.max_sensor_range == 625

    b.remove_sensor(s)

    assert b.max_sensor_range == 0

    with pytest.raises(RuntimeError):
        b.remove_sensor(s)


def test_perf(physics):
    s = physics.ActiveRadar(power=50, sensitivity=2)

    assert s.max_range == 625

    s.perf = 0.5

    assert s.max_range == 625 / 2

    s = physics.PassiveRadar(sensitivity=3)

    assert s.max_range == 50 / 3

    s.perf = 0.5

    assert s.max_range == 50 / 6

    s = physics.PassiveThermal(sensitivity=3)

    assert s.max_range == 2500 / 3

    s.perf = 0.5

    assert s.max_range == 2500 / 6
