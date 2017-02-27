/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
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

#ifndef SHAPE_HPP
#define SHAPE_HPP

#include "aabb.hpp"
#include "collisions.hpp"
#include "vec.hpp"

class Body;
class Circle;
class Polygon;

class Shape {
public:
    Body* body = nullptr;
    double mass = 0;
    Vec position;

    Shape(double mass_=0, const Vec& position_={});
    virtual ~Shape();
    virtual AABB aabb() const = 0;

    // Double dispatch
    virtual CollisionTimeResult collide(const Shape* other, double end_time) const = 0;
    virtual CollisionTimeResult collide(const Circle* other, double end_time) const = 0;
//     virtual CollisionTimeResult collide(const Polygon* other) const = 0;
};

#endif // SHAPE_HPP
