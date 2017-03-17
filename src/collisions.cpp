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
#include "polygon.hpp"
#include "body.hpp"
#include <iostream>

using namespace std;


CollisionTimeResult collideCircleCircle(const Circle* acircle, const Circle* bcircle, double end_time, bool entering) {
    auto abody = acircle->body;
    auto bbody = bcircle->body;
    auto apos = abody->position() + acircle->position;
    auto bpos = bbody->position() - bcircle->position;
    auto pos_diff = apos - bpos;
    auto radii = acircle->radius + bcircle->radius;
    auto dist = pos_diff.abs();
    auto vel_diff = abody->velocity() - bbody->velocity();

    double t;

    if (((pos_diff.dot(vel_diff) < 0) == entering) && ((entering && dist <= radii) || (!entering && dist > radii))) {
        t = 0;
    }
    else {
        auto a = vel_diff.squared();
        auto b = 2 * pos_diff.dot(vel_diff);
        auto c = pos_diff.squared() - radii * radii;
        auto discriminant = b * b - 4 * a * c;

        if (discriminant < 0 || a == 0) {
            return {};
        }

        if (entering) {
            t = (-b - sqrt(discriminant)) / (2 * a);
        }
        else {
            t = (-b + sqrt(discriminant)) / (2 * a);
        }
        if (0 > t || t > end_time) {
            return {};
        }
    }
    auto col_apos = (apos + abody->velocity() * t);
    auto col_bpos = (bpos + bbody->velocity() * t);
    auto touch_point = (col_apos * bcircle->radius + col_bpos * acircle->radius) / radii;
    auto norm = col_bpos - col_apos;
    return {abody, bbody, t, touch_point, norm ? norm.norm() : Vec{1, 0}, entering};
}


inline pair<double, double> axis_proj_max_min(const vector<Vec>& points, Vec axis) {
    double max, min;
    max = min = points[0].dot(axis);
    for (unsigned int i = 1; i < points.size(); ++i) {
        auto dot = points[i].dot(axis);
        if (dot < min) {
            min = dot;
        }
        else if (dot > max) {
            max = dot;
        }
    }
    return {max, min};
}

CollisionTimeResult collideCirclePolygon(const Circle* acircle, const Polygon* bpoly, double end_time, bool entering) {
    auto abody = acircle->body;
    auto bbody = bpoly->body;
    auto apos = abody->position() + acircle->position;
    auto bpos = bbody->position() + bpoly->position;
    auto pos_diff = apos - bpos;
    auto vel_diff = abody->velocity() - bbody->velocity();

    double t = -1;
    Vec max_axis;

    for (int i = -1; i < static_cast<int>(bpoly->points.size()); ++i) {
        double local_t;
        auto axis = (i < 0)? pos_diff.norm() : (bpoly->points[(i + 1) % bpoly->points.size()] - bpoly->points[i]).perp();
        auto vel_diff_proj = vel_diff.dot(axis);
        auto pos_diff_proj = pos_diff.dot(axis);
        cout << axis << vel_diff_proj << " " << pos_diff_proj << endl;
        if (!vel_diff_proj) {
            continue;
        }
        auto max_min = axis_proj_max_min(bpoly->points, axis);
        cout << "DIST" << (max_min.first - pos_diff_proj + acircle->radius) << endl;
        if (pos_diff_proj > 0) {
            local_t = max(0.0, (max_min.first - pos_diff_proj + acircle->radius) / vel_diff_proj);
        }
        else {
            local_t = max(0.0, (max_min.second - pos_diff_proj - acircle->radius) / vel_diff_proj);
        }
        if (local_t > t) {
            t = local_t;
            max_axis = axis;
        }
    }
    if (t > end_time || t < 0) {
        return {};
    }
    auto touch_point = (apos + abody->velocity() * t) - max_axis * acircle->radius;
    return {abody, bbody, t, touch_point, -max_axis, entering};
}

CollisionTimeResult collidePolygonPolygon(const Polygon* a, const Polygon* b, double end_time, bool entering) {

}

bool immediateCollideCirclePolygon(const Circle* acircle, const Polygon* bpoly) {
    auto abody = acircle->body;
    auto bbody = bpoly->body;
    auto apos = abody->position() + acircle->position;
    auto bpos = bbody->position() + bpoly->position;
    auto pos_diff = apos - bpos;
    for (unsigned int i = 0; i < bpoly->points.size(); ++i) {
        auto axis = (bpoly->points[(i + 1) % bpoly->points.size()] - bpoly->points[i]).perp();
        auto max_min = axis_proj_max_min(bpoly->points, axis);
        auto pos_diff_proj = pos_diff.dot(axis);
        if (max_min.first - pos_diff_proj < -acircle->radius || max_min.second - pos_diff_proj > acircle->radius) {
            return false;
        }
    }
    auto axis = pos_diff.norm();
    auto max_min = axis_proj_max_min(bpoly->points, axis);
    auto pos_diff_proj = pos_diff.abs();
    return !(max_min.first - pos_diff_proj < -acircle->radius || max_min.second - pos_diff_proj > -acircle->radius);
}

bool immediateCollidePolygonPolygon(const Polygon* a, const Polygon* b) {

}

CollisionResult collisionResult(const CollisionTimeResult& cr, const CollisionParameters& params) {
    Vec a_vel = cr.a->velocity().proj(cr.normal);
    Vec b_vel = cr.b->velocity().proj(cr.normal);
    double m_a = cr.a->mass();
    double m_b = cr.b->mass();
    Vec closing_velocity = b_vel - a_vel;
    Vec impulse = (m_a * m_b * closing_velocity * (1 + params.restitution)) / (m_a + m_b);
    if (impulse.abs() > params.transition_impulse) {
        impulse = impulse * params.transition_reduction + impulse.norm() * params.transition_impulse * (1 - params.transition_reduction);
    }
    else if (!impulse) {
        impulse = 0.00001 * cr.normal;
    }
    return {impulse, closing_velocity};
}
