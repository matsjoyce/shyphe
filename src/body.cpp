/*
 * shyphe - Stiff HIgh velocity PHysics Engine
 * Copyright (C) 2017 Matthew Joyce matsjoyce@gmail.com
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

#include <algorithm>
#include <cmath>
#include <numeric>

using namespace std;
using namespace shyphe;

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

void Body::update(double time) {
    if (!time) {
        return;
    }
    if (time < 0) {
        throw runtime_error("time cannot be negative, use state() and reset()");
    }

    auto angular_acceleration = (_local_torque + _global_torque) / momentOfInertia();

    // Use trapezium rule to integrate local forces

    int strips = ceil(time * 100);
    auto vel_accumulator = _local_force.rotate(_angle);
    auto pos_accumulator = Vec{};

    for (auto i = 1; i != strips; ++i) {
        auto t = i / static_cast<double>(strips) * time;
        auto angle = _angle + _angular_velocity * t + angular_acceleration * t * t / 2;
        auto impulse = _local_force.rotate(angle);
        vel_accumulator += impulse;
        pos_accumulator += vel_accumulator;
        vel_accumulator += impulse;
    }

    _angle = norm_rad(_angle + _angular_velocity * time + angular_acceleration * time * time / 2);
    _angular_velocity = _angular_velocity + angular_acceleration * time;

    vel_accumulator += _local_force.rotate(_angle);
    pos_accumulator += vel_accumulator / 2;
    vel_accumulator /= strips * 2;
    pos_accumulator /= strips * strips * 2;

    _position += _velocity * time + (pos_accumulator + _global_force / 2) * time * time / mass();
    _velocity += (vel_accumulator + _global_force) * time / mass();
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
    _local_force += impulse;
    _local_torque -= position.perp().dot(impulse);
}

void Body::clearLocalForces() {
    _local_force = {0, 0};
    _local_torque = 0;
}

void Body::applyGlobalForce(Vec impulse, Vec position) {
    _global_force += impulse;
    _global_torque -= position.perp().dot(impulse);
}

void Body::clearGlobalForces() {
    _global_force = {0, 0};
    _global_torque = 0;
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

BodyState Body::state() const {
    return {_position, _velocity,
            _local_force, _global_force,
            _local_torque, _global_torque,
            _angle, _angular_velocity};
}

void Body::reset(BodyState state) {
    _position = state.position;
    _velocity = state.velocity;
    _local_force = state.local_force;
    _global_force = state.global_force;
    _local_torque = state.local_torque;
    _global_torque = state.global_torque;
    _angle = state.angle;
    _angular_velocity = state.angular_velocity;
}
