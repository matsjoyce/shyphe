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
    Body(const Vec& position_={}, const Vec& velocity_={},
         double angle_=0, double angular_velocity_=0, int side_=0);
    virtual ~Body() = default;

    AABB aabb() const;
    double mass() const;
    inline const Vec& position() const {
        return _position;
    }
    inline const Vec& velocity() const {
        return _velocity;
    }
    inline double angle() const {
        return _angle;
    }
    inline double angularVelocity() const {
        return _angular_velocity;
    }
    inline int side() const {
        return _side;
    }
    inline const std::vector<SensedObject>& sensorView() const {
        return _sensor_view;
    }

    void changeSide(int new_side);
    void teleport(const Vec& to);

    void applyImpulse(Vec impulse, Vec position);
    void applyLocalForce(Vec force, Vec position);
    void applyGlobalForce(Vec force, Vec position);
    void clearLocalForces();
    void clearGlobalForces();

    void HACK_setAngularVelocity(double vel);

    Signature signature();
    void updatePosition(double time);
    void updateVelocity(double time);
    void addShape(Shape* shape);
    void removeShape(Shape* shape);
    void addSensor(Sensor* shape);
    void removeSensor(Sensor* shape);
    CollisionTimeResult collide(Body* other, double end_time, bool ignore_initial) const;
    double distanceBetween(Body* other) const;
    double maxSensorRange() const;
private:
    Vec _position, _velocity;
    Vec _local_forces = {}, _global_forces = {};
    double _angle, _angular_velocity;
    int _side;
    std::vector<SensedObject> _sensor_view;

    std::vector<Shape*> shapes;
    std::vector<Sensor*> sensors;

    friend class World;
};

#endif // BODY_HPP
