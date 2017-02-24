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

class Body;
class Circle;
class Polygon;

struct CollisionTimeResult {
    constexpr CollisionTimeResult(Body* a_, Body* b_, double time_, Vec tp, Vec norm): a(a_), b(b_), time(time_), touch_point(tp), normal(norm) {
    }

    constexpr CollisionTimeResult() {
    }

    Body* a = nullptr;
    Body* b = nullptr;
    double time = -1;
    Vec touch_point = {0, 0};
    Vec normal = {0, 0};
};

CollisionTimeResult collideCircleCircle(const Circle* a, const Circle* b, double end_time);

struct CollisionResult {
    Vec impulse;
    Vec closing_velocity;
};

CollisionResult collisionResult(const CollisionTimeResult& cr, double E, double transition_impulse, double transition_reduction);

#endif // COLLISIONS_HPP
