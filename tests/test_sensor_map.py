import physics
import pytest


def test_simple():
    b1 = physics.Body(position=(0, 0))
    b1.add_sensor(physics.ActiveRadar(power=50, sensitivity=1))

    b2 = physics.Body(position=(10, 10))
    s = physics.MassShape()
    s.signature.radar_cross_section = 20
    b2.add_shape(s)

    sm = physics.SensorMap()
    sm.add_body(b1)
    sm.add_body(b2)

    sm.reset()
    sm.scan(b1)

    assert len(b1.sensor_view) == 1

    so = b1.sensor_view[0]

    assert so.body is b2
    assert so.intensity == 20
    assert so.position.as_tuple() == (10, 10)
    assert so.velocity.as_tuple() == (0, 0)
    assert so.side == physics.Side.neutral
