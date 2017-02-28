/*
 * LunyFringe.core.engine.physics - rigid body physics engine
 * Copyright (C) 2016 Matthew Joyce matsjoyce@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef BODY_HPP
#define BODY_HPP

#include <vector>

#include "vec.hpp"
#include "aabb.hpp"
#include "collisions.hpp"
#include "shape.hpp"
#include "sensor.hpp"

class Body {
public:
    Vec position, velocity, acceleration;
    double angle, angular_velocity, angular_acceleration;
    int side;
    std::vector<SensedObject> sensor_view;

    Body(const Vec& position_={}, const Vec& velocity_={}, const Vec& acceleration_={},
         double angle_=0, double angular_velocity_=0, double angular_acceleration_=0, int side_=0);
    virtual ~Body();

    AABB aabb() const;
    double mass() const;
    Signature signature();
    void updatePosition(double time);
    void updateVelocity(double time);
    void applyImpulse(Vec impulse, Vec position);
    void addShape(Shape* shape);
    void removeShape(Shape* shape);
    void addSensor(Sensor* shape);
    void removeSensor(Sensor* shape);
    CollisionTimeResult collide(Body* other, double end_time) const;
    bool immediate_collide(Body* other) const;
    double maxSensorRange() const;
private:
    std::vector<Shape*> shapes;
    std::vector<Sensor*> sensors;

    friend class SensorMap;
};

#endif // BODY_HPP
