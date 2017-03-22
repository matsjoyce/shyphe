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

#ifndef COLLISIONS_HPP
#define COLLISIONS_HPP

#include "vec.hpp"

class Shape;
class Body;
class Circle;
class Polygon;

struct DistanceResult {
    constexpr DistanceResult(double dist, Vec a, Vec b, Vec norm): distance(dist), a_point(a), b_point(b), normal(norm) {
    }

    constexpr DistanceResult() {
    }

    double distance = 0;
    Vec a_point, b_point, normal;
};

struct CollisionTimeResult {
    constexpr CollisionTimeResult(Body* a_, Body* b_, Shape* a_shape_, Shape* b_shape_,
                                  double time_, Vec tp, Vec norm): a(a_),
                                                                   b(b_),
                                                                   a_shape(a_shape_),
                                                                   b_shape(b_shape_),
                                                                   time(time_),
                                                                   touch_point(tp),
                                                                   normal(norm) {
    }

    constexpr CollisionTimeResult() {
    }

    Body* a = nullptr;
    Body* b = nullptr;
    Shape* a_shape = nullptr;
    Shape* b_shape = nullptr;
    double time = -1;
    Vec touch_point = {0, 0};
    Vec normal = {0, 0};
};

CollisionTimeResult collideShapes(Shape* a, Shape* b, double end_time, bool ignore_initial);

DistanceResult distanceBetweenCircleCircle(const Shape&, const Body&, const Shape&, const Body&);
DistanceResult distanceBetweenCirclePolygon(const Shape&, const Body&, const Shape&, const Body&);
DistanceResult distanceBetweenPolygonCircle(const Shape&, const Body&, const Shape&, const Body&);
DistanceResult distanceBetweenPolygonPolygon(const Shape&, const Body&, const Shape&, const Body&);
DistanceResult distanceBetween(const Shape*, const Shape*);

struct CollisionResult {
    Vec impulse;
    Vec closing_velocity;
};

struct CollisionParameters {
    constexpr CollisionParameters(double restitution_): restitution(restitution_) {
    }

    double restitution;
};

CollisionResult collisionResult(const CollisionTimeResult& cr, const CollisionParameters& params);

#endif // COLLISIONS_HPP
