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
#include <iostream>

using namespace std;

CollisionTimeResult collideCircleCircle(const Circle* acircle, const Circle* bcircle, double end_time) {
    auto abody = acircle->body;
    auto bbody = bcircle->body;
    auto apos = abody->position + acircle->position;
    auto bpos = bbody->position - bcircle->position;
    auto pos_diff = apos - bpos;
    auto radii = acircle->radius + bcircle->radius;

    if (pos_diff.abs() < radii) {
        return {abody, bbody, 0, (apos * bcircle->radius + bpos * acircle->radius) / radii, (bpos - apos).norm()};
    }

    auto vel_diff = abody->velocity - bbody->velocity;

    auto a = vel_diff.squared();
    auto b = 2 * pos_diff.dot(vel_diff);
    auto c = pos_diff.squared() - radii * radii;
    auto discriminant = b * b - 4 * a * c;

    if (discriminant < 0 || a == 0) {
        return {};
    }

    auto t0 = (-b - sqrt(discriminant)) / (2 * a);
    auto t1 = (-b + sqrt(discriminant)) / (2 * a);

    if (t0 > end_time) {
        return {};
    }
    else if (t0 >= 0) {
        t1 = t0;
    }
    else if (0 > t1 || t1 > end_time) {
        return {};
    }
    auto col_apos = (apos + abody->velocity * t1);
    auto col_bpos = (bpos + bbody->velocity * t1);
    auto touch_point = (col_apos * bcircle->radius + col_bpos * acircle->radius) / radii;
    return {abody, bbody, t1, touch_point, (col_bpos - col_apos).norm()};
}

CollisionResult collisionResult(const CollisionTimeResult& cr, double restitution, double transition_impulse, double transition_reduction) {
    Vec a_vel = cr.a->velocity.proj(cr.normal);
    Vec b_vel = cr.b->velocity.proj(cr.normal);
    double m_a = cr.a->mass();
    double m_b = cr.b->mass();
    Vec closing_velocity = b_vel - a_vel;
    Vec impulse = (m_a * m_b * closing_velocity * (1 + restitution)) / (m_a + m_b);
    if (impulse.abs() > transition_impulse) {
        impulse = impulse * transition_reduction + impulse.norm() * transition_impulse * (1 - transition_reduction);
    }
    return {impulse, closing_velocity};
}
