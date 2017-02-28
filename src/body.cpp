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

Body:: Body(const Vec& position_/*={}*/, const Vec& velocity_/*={}*/, const Vec& acceleration_/*={}*/,
            double angle_/*=0*/, double angular_velocity_/*=0*/, double angular_acceleration_/*=0*/,
            int side_/*=0*/) : position(position_),
                               velocity(velocity_),
                               acceleration(acceleration_),
                               angle(angle_),
                               angular_velocity(angular_velocity_),
                               angular_acceleration(angular_acceleration_),
                               side(side_) {
}

Body::~Body() {
}

AABB Body::aabb() const {
    auto iter = shapes.begin();
    auto end = shapes.end();
    AABB aabb = {0, 0, 0, 0};
    for (; iter != end; ++iter) {
        if ((*iter)->canCollide()) {
            aabb = (*iter)->aabb() + (*iter)->position;
            break;
        }
    }
    if (iter == end) {
        return {0, 0, 0, 0};
    }
    for (; iter != end; ++iter) {
        if ((*iter)->canCollide()) {
            aabb &= (*iter)->aabb() + (*iter)->position;
        }
    }
    return aabb;
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

void Body::updatePosition(double time) {
    position += velocity * time;
}

void Body::updateVelocity(double time) {
    velocity += acceleration * time;
    angular_velocity += angular_acceleration * time;
    angle = norm_rad(angle + angular_velocity * time);
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

CollisionTimeResult Body::collide(Body* other, double end_time) const {
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
            auto collr = my_shape->collide(their_shape, end_time);
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

bool Body::immediate_collide(Body* other) const {
    for (const auto my_shape : shapes) {
        if (!my_shape->canCollide()) {
            continue;
        }
        for (const auto their_shape : other->shapes) {
            if (!their_shape->canCollide()) {
                continue;
            }
            if (my_shape->immediate_collide(their_shape)) {
                return true;
            }
        }
    }
    return false;
}

void Body::applyImpulse(Vec impulse, Vec position) {
    velocity += impulse / mass();
}

double Body::maxSensorRange() const {
    double m = 0;
    for (const auto& sensor : sensors) {
        m = max(m, sensor->maxRange());
    }
    return m;
}
