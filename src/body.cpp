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

#include <cmath>

using namespace std;

AABB Body::aabb() const {
    auto iter = shapes.begin();
    auto end = shapes.end();
    if (iter == end) {
        return {0, 0, 0, 0};
    }
    auto aabb = (*iter)->aabb() + (*iter)->position;
    for (; iter != end; ++iter) {
        aabb &= (*iter)->aabb() + (*iter)->position;
    }
    return aabb;
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

CollisionTimeResult Body::collide(Body* other, double end_time) {
    auto soonest = CollisionTimeResult{};
    soonest.time = end_time + 1;
    for (const auto my_shape : shapes) {
        for (const auto their_shape : other->shapes) {
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
