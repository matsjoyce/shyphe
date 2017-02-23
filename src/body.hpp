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
#include "shape.hpp"
#include "collisions.hpp"

class Body {
public:
    virtual ~Body();

    Vec position, velocity, acceleration;
    double angle, angular_velocity, angular_acceleration;

    AABB aabb() const;
    void updatePosition(double time);
    void updateVelocity(double time);
    void addShape(Shape* shape);
    CollisionTimeResult collide(Body* other, double end_time);
private:
    std::vector<Shape*> shapes;
};

#endif // BODY_HPP
