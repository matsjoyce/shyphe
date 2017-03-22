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

#include "body.hpp"
#include "utils.hpp"
#include "shape.hpp"
#include "sensor.hpp"

#include <cmath>
#include <numeric>

using namespace std;

Body::Body(const Vec& position_/*={}*/, const Vec& velocity_/*={}*/,
           double angle_/*=0*/, double angular_velocity_/*=0*/,
           int side_/*=0*/) : _position(position_),
                              _velocity(velocity_),
                              _angle(angle_),
                              _angular_velocity(angular_velocity_),
                              _side(side_) {
}

AABB aabbAtAngle(const vector<Shape*>& shapes, double angle) {
    auto iter = shapes.begin();
    auto end = shapes.end();
    AABB aabb = {0, 0, 0, 0};
    for (; iter != end; ++iter) {
        if ((*iter)->canCollide()) {
            aabb = (*iter)->aabb(angle) + (*iter)->position.rotate(angle);
            break;
        }
    }
    if (iter == end) {
        return {0, 0, 0, 0};
    }
    for (; iter != end; ++iter) {
        if ((*iter)->canCollide()) {
            aabb &= (*iter)->aabb(angle) + (*iter)->position.rotate(angle);
        }
    }
    return aabb;
}

AABB Body::aabb(double time) const {
    AABB aabb = aabbAtAngle(shapes, _angle);
    if (_angular_velocity) {
        auto end_angle = _angle + _angular_velocity * time;
        aabb &= aabbAtAngle(shapes, end_angle);
        int extreme_start = ceil(_angle / hpi());
        int extreme_range = (end_angle - _angle) / hpi();
        int extreme_end;
        if (extreme_range < 0) {
            extreme_end = extreme_start;
            extreme_start -= floor(abs(extreme_range));
        }
        else {
            extreme_end = extreme_start + floor(abs(extreme_range));
        }

        for (; extreme_start <= extreme_end; ++extreme_start) {
            aabb &= aabbAtAngle(shapes, extreme_start * hpi());
        }
    }
    return aabb & (aabb + _velocity * time);
}

Signature Body::signature() {
    Signature sig;
    for (const auto& shape : shapes) {
        sig.radar_cross_section += shape->signature.radar_cross_section;
        sig.radar_emissions += shape->signature.radar_emissions;
        sig.thermal_emissions += shape->signature.thermal_emissions;
    }
    return sig;
}

double Body::mass() const {
    return accumulate(shapes.begin(), shapes.end(), 0.0, [](double acc, const Shape* shape){return acc + shape->mass;});
}

double Body::momentOfInertia() const {
    double moi = 0;
    for (const auto& shape : shapes) {
        moi += shape->momentOfInertia() + shape->mass * shape->position.squared();
    }
    return moi;
}

void Body::updatePosition(double time) {
    _position += _velocity * time;
    _angle = norm_rad(_angle + _angular_velocity * time);
}

void Body::updateVelocity(double time) {
    _velocity += (_local_forces.rotate(_angle) + _global_forces) * time / mass();
//     _angular_velocity += angular_acceleration * time;
}

void Body::addShape(Shape* shape) {
    if (shape->body) {
        throw runtime_error("Shape already has body");
    }
    shapes.push_back(shape);
    shape->body = this;
}

void Body::removeShape(Shape* shape) {
    if (shape->body != this) {
        throw runtime_error("Shape not attached to this body");
    }
    shapes.erase(remove(shapes.begin(), shapes.end(), shape), shapes.end());
    shape->body = nullptr;
}

void Body::addSensor(Sensor* sensor) {
    if (sensor->body) {
        throw runtime_error("Sensor already has body");
    }
    sensors.push_back(sensor);
    sensor->body = this;
}

void Body::removeSensor(Sensor* sensor) {
    if (sensor->body != this) {
        throw runtime_error("Sensor not attached to this body");
    }
    sensors.erase(remove(sensors.begin(), sensors.end(), sensor), sensors.end());
    sensor->body = nullptr;
}

CollisionTimeResult Body::collide(Body* other, double end_time, bool ignore_initial) const {
    auto soonest = CollisionTimeResult{};
    soonest.time = end_time + 1;
    for (const auto my_shape : shapes) {
        if (!my_shape->canCollide()) {
            continue;
        }
        for (const auto their_shape : other->shapes) {
            if (!their_shape->canCollide()) {
                continue;
            }
            auto collr = collideShapes(my_shape, their_shape, end_time, ignore_initial);
            if (collr.time != -1 && collr.time < soonest.time) {
                soonest = move(collr);
            }
        }
    }
    if (soonest.time < end_time) {
        return soonest;
    }
    soonest.time = -1;
    return soonest;
}

double Body::distanceBetween(Body* other) const {
    bool initial = true;
    double dist = (other->_position - _position).abs();
    for (const auto my_shape : shapes) {
        if (!my_shape->canCollide()) {
            continue;
        }
        for (const auto their_shape : other->shapes) {
            if (!their_shape->canCollide()) {
                continue;
            }
            auto d = ::distanceBetween(my_shape, their_shape).distance;
            if (initial || d < dist) {
                dist = d;
                initial = false;
            }
        }
    }
    return dist;
}

void Body::applyImpulse(Vec impulse, Vec position) {
    _velocity += impulse / mass();
    _angular_velocity -= position.perp().dot(impulse) / momentOfInertia();
}

void Body::applyLocalForce(Vec impulse, Vec position) {
    _local_forces += impulse;
}

void Body::clearLocalForces() {
    _local_forces = {0, 0};
}

void Body::applyGlobalForce(Vec impulse, Vec position) {
    _global_forces += impulse;
}

void Body::clearGlobalForces() {
    _global_forces = {0, 0};
}

void Body::teleport(const Vec& to) {
    _position = to;
}

void Body::changeSide(int side) {
    _side = side;
}

double Body::maxSensorRange() const {
    double m = 0;
    for (const auto& sensor : sensors) {
        m = max(m, sensor->maxRange());
    }
    return m;
}

void Body::HACK_setAngularVelocity(double vel) {
    _angular_velocity = vel;
}
