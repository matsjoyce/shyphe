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

AABB aabbAtAngle(const vector<shared_ptr<Shape>>& shapes, double angle) {
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
    AABB aabb = aabbAtAngle(_shapes, _angle);
    if (_angular_velocity) {
        auto end_angle = _angle + _angular_velocity * time;
        aabb &= aabbAtAngle(_shapes, end_angle);
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
            aabb &= aabbAtAngle(_shapes, extreme_start * hpi());
        }
    }
    return aabb & (aabb + _velocity * time);
}

Signature Body::signature() {
    Signature sig;
    for (const auto& shape : _shapes) {
        sig.radar_cross_section += shape->signature.radar_cross_section;
        sig.radar_emissions += shape->signature.radar_emissions;
        sig.thermal_emissions += shape->signature.thermal_emissions;
    }
    return sig;
}

double Body::mass() const {
    return accumulate(_shapes.begin(), _shapes.end(), 0.0, [](double acc, const shared_ptr<Shape>& shape){return acc + shape->mass;});
}

double Body::momentOfInertia() const {
    double moi = 0;
    for (const auto& shape : _shapes) {
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

void Body::addShape(shared_ptr<Shape> shape) {
    _shapes.push_back(shape);
}

void Body::removeShape(shared_ptr<Shape> shape) {
    _shapes.erase(remove(_shapes.begin(), _shapes.end(), shape), _shapes.end());
}

void Body::addSensor(shared_ptr<Sensor> sensor) {
    _sensors.push_back(sensor);
}

void Body::removeSensor(shared_ptr<Sensor> sensor) {
    _sensors.erase(remove(_sensors.begin(), _sensors.end(), sensor), _sensors.end());
}

tuple<CollisionTimeResult, Shape*, Shape*> Body::collide(Body* other, double end_time, bool ignore_initial) const {
    auto soonest = CollisionTimeResult{};
    Shape* a;
    Shape* b;
    soonest.time = end_time + 1;
    for (const auto my_shape : _shapes) {
        if (!my_shape->canCollide()) {
            continue;
        }
        for (const auto their_shape : other->_shapes) {
            if (!their_shape->canCollide()) {
                continue;
            }
            auto collr = collideShapes(*my_shape, *this, *their_shape, *other, end_time, ignore_initial);
            if (collr.time != -1 && collr.time < soonest.time) {
                soonest = move(collr);
                a = my_shape.get();
                b = their_shape.get();
            }
        }
    }
    if (soonest.time < end_time) {
        return {soonest, a, b};
    }
    soonest.time = -1;
    return {soonest, nullptr, nullptr};
}

double Body::distanceBetween(Body* other) const {
    bool initial = true;
    double dist = (other->_position - _position).abs();
    for (const auto my_shape : _shapes) {
        if (!my_shape->canCollide()) {
            continue;
        }
        for (const auto their_shape : other->_shapes) {
            if (!their_shape->canCollide()) {
                continue;
            }
            auto d = ::distanceBetween(*my_shape, *this, *their_shape, *other).distance;
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
    for (const auto& sensor : _sensors) {
        m = max(m, sensor->maxRange());
    }
    return m;
}

void Body::HACK_setAngularVelocity(double vel) {
    _angular_velocity = vel;
}
