import physics
import pytest


def test_active_radar():
    b1 = physics.Body(position=(0, 0))
    b1.add_sensor(physics.ActiveRadar(power=50, sensitivity=1))

    b2 = physics.Body(position=(10, 10))
    b2.add_shape(physics.MassShape(radar_cross_section=20))

    b3 = physics.Body(position=(10, -10))
    b3.add_shape(physics.MassShape(radar_emissions=10, thermal_emissions=10))

    w = physics.World()
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame(1)
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.intensity == 20
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == physics.Side.neutral


def test_passive_radar():
    b1 = physics.Body(position=(0, 0))
    b1.add_sensor(physics.PassiveRadar(sensitivity=1))

    b2 = physics.Body(position=(10, 10))
    b2.add_shape(physics.MassShape(radar_emissions=25))

    b3 = physics.Body(position=(10, -10))
    b3.add_shape(physics.MassShape(radar_cross_section=10, thermal_emissions=10))

    w = physics.World()
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame(1)
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.intensity == 25
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == physics.Side.unknown


def test_passive_thermal():
    b1 = physics.Body(position=(0, 0))
    b1.add_sensor(physics.PassiveThermal(sensitivity=1))

    b2 = physics.Body(position=(10, 10))
    b2.add_shape(physics.MassShape(thermal_emissions=15))

    b3 = physics.Body(position=(10, -10))
    b3.add_shape(physics.MassShape(radar_cross_section=10, radar_emissions=10))

    w = physics.World()
    w.add_body(b1)
    w.add_body(b2)
    w.add_body(b3)

    w.begin_frame(1)
    w.end_frame()

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.intensity == 15
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == physics.Side.unknown


def test_tracking():
    b = physics.Body(position=(0, 0), side=1)
    b.add_sensor(physics.ActiveRadar(power=50, sensitivity=2))
    b.add_shape(physics.MassShape(radar_cross_section=50))

    b2 = physics.Body(position=(10, 0), velocity=(5, 0), side=1)
    b2.add_sensor(physics.ActiveRadar(power=50, sensitivity=2))
    b2.add_shape(physics.MassShape(radar_cross_section=50))

    w = physics.World()
    w.add_body(b)
    w.add_body(b2)

    w.begin_frame(1)
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    sr = b.sensor_view[0]
    assert sr.side == physics.Side.friendly
    assert sr.position.as_tuple() == (10, 0)
    assert not sr.velocity
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == physics.Side.friendly
    assert sr.position.as_tuple() == (-10, 0)
    assert not sr.velocity
    assert sr.body == b

    assert b2.position.as_tuple() == (15, 0)

    b2.side = 2

    w.begin_frame(1)
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    assert b.sensor_view[0] == b.sensor_view[0]
    assert b.sensor_view[0] != b2.sensor_view[0]

    sr = b.sensor_view[0]
    assert sr.side == physics.Side.enemy
    assert sr.position.as_tuple() == (15, 0)
    assert sr.velocity.as_tuple() == (5, 0)
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == physics.Side.enemy
    assert sr.position.as_tuple() == (-15, 0)
    assert sr.velocity.as_tuple() == (-5, 0)
    assert sr.body == b

    assert b2.position.as_tuple() == (20, 0)

    b2.position = (-30, 60)

    w.begin_frame(1)
    w.end_frame()

    assert list(b.sensor_view)
    assert list(b2.sensor_view)

    sr = b.sensor_view[0]
    assert sr.side == physics.Side.enemy
    assert sr.position.as_tuple() == (-30, 60)
    assert sr.velocity.as_tuple() == (-55, 60)
    assert sr.body == b2

    sr = b2.sensor_view[0]
    assert sr.side == physics.Side.enemy
    assert sr.position.as_tuple() == (30, -60)
    assert sr.velocity.as_tuple() == (55, -60)
    assert sr.body == b


