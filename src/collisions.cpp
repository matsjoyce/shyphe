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

#include "collisions.hpp"
#include "shape.hpp"
#include "circle.hpp"
#include "body.hpp"

using namespace std;

constexpr CollisionTimeResult noCollision() {
    return {nullptr, nullptr, -1, {0, 0}};
}

CollisionTimeResult collide_circle_circle(const Circle* acircle, const Circle* bcircle, double end_time) {
    auto abody = acircle->body;
    auto bbody = bcircle->body;
    auto apos = abody->position + acircle->position;
    auto bpos = bbody->position - bcircle->position;
    auto pos_diff = apos - bpos;
    auto vel_diff = abody->velocity - bbody->velocity;

    auto radii = acircle->radius + bcircle->radius;
    auto a = vel_diff.squared();
    auto b = 2 * pos_diff.dot(vel_diff);
    auto c = pos_diff.squared() - radii * radii;
    auto discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return noCollision();
    }

    auto t0 = (-b - sqrt(discriminant)) / (2 * a);
    auto t1 = (-b + sqrt(discriminant)) / (2 * a);

    if (t0 > end_time) {
        return noCollision();
    }
    else if (t0 > 0) {
        t1 = t0;
    }
    else if (t1 > end_time) {
        return noCollision();
    }
    auto touch_point = ((apos + abody->velocity * t1) * bcircle->radius)
                        + ((bpos + bbody->velocity * t1) * acircle->radius) / radii;
    return {abody, bbody, t1, touch_point};
}
