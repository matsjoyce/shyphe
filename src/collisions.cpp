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
#include <typeindex>
#include <map>
#include <cmath>
#include <tuple>

using namespace std;

const double COLLISION_LIMIT = 1e-8;
typedef DistanceResult (*DistanceDispatch)(const Shape&, const Body&, const Shape&, const Body&);

const map<pair<type_index, type_index>, DistanceDispatch> DISPATCH_TABLE = {
    {{type_index(typeid(Circle)), type_index(typeid(Circle))}, &distanceBetweenCircleCircle},
    {{type_index(typeid(Circle)), type_index(typeid(Polygon))}, &distanceBetweenCirclePolygon},
    {{type_index(typeid(Polygon)), type_index(typeid(Circle))}, &distanceBetweenPolygonCircle},
    {{type_index(typeid(Polygon)), type_index(typeid(Polygon))}, &distanceBetweenPolygonPolygon}
};

DistanceResult distanceBetween(const Shape* a, const Shape* b) {
    DistanceDispatch dist_func = DISPATCH_TABLE.at(make_pair(a->shape_type(), b->shape_type()));
    return dist_func(*a, *a->body, *b, *b->body);
}

CollisionTimeResult collideShapes(const Shape* a, const Shape* b, double end_time, bool ignore_initial) {
    DistanceDispatch dist_func = DISPATCH_TABLE.at(make_pair(a->shape_type(), b->shape_type()));
    Body abody = *a->body, bbody = *b->body;
    auto vel_diff = abody.velocity() - bbody.velocity();
    DistanceResult current_distance;
    double time = 0;
    unsigned int iteration = 0;
    while (iteration < 20) {
        current_distance = dist_func(*a, abody, *b, bbody);

        double vel = vel_diff.dot(current_distance.normal)
                     + (a->position.abs() + a->boundingRadius()) * a->body->angularVelocity()
                     + (b->position.abs() + b->boundingRadius()) * b->body->angularVelocity();
        double add_time = current_distance.distance / vel;
        time += add_time;
        if (time > end_time) {
            return {};
        }
        auto cmp = (ignore_initial && !iteration) ? current_distance.distance : abs(current_distance.distance);
        if (cmp < COLLISION_LIMIT) {
            auto vel_at = vel_diff + (current_distance.a_point + a->position).perp() * a->body->angularVelocity() - (current_distance.b_point + b->position).perp() * b->body->angularVelocity();
            if (vel_at.dot(current_distance.normal) > 0 && (iteration || !ignore_initial)) {
                break;
            }
            else {
                add_time += COLLISION_LIMIT * 3 / vel_at;
                time += COLLISION_LIMIT * 3 / vel_at;
            }
        }
        if (vel <= 0 || time < 0) {
            return {};
        }

        abody.updatePosition(add_time);
        bbody.updatePosition(add_time);

        ++iteration;
    }
    return {a->body, b->body, time, (current_distance.a_point + current_distance.b_point) / 2.0, current_distance.normal};
}

DistanceResult distanceBetweenCircleCircle(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_circle = dynamic_cast<const Circle&>(b);

    auto apos = a_body.position() + a_circle.position;
    auto bpos = b_body.position() + b_circle.position;
    auto ray = bpos - apos;
    auto norm = ray ? ray.norm() : Vec{1, 0};
    return {ray.abs() - a_circle.radius - b_circle.radius, apos + a_circle.radius * norm, bpos - b_circle.radius * norm, norm};
}

DistanceResult distanceBetweenCirclePolygon(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_poly = dynamic_cast<const Polygon&>(b);

    auto apos = a_body.position() + a_circle.position;
    auto bpos = b_body.position() + b_poly.position;
    DistanceResult d;
    for (unsigned int i = 0; i < b_poly.points.size(); ++i) {
        auto p1 = b_poly.points[i], p2 = b_poly.points[(i + 1) % b_poly.points.size()];
        auto u = p2 - p1, v = apos - p1 - bpos;
        auto determinant = u.dot(v) / u.squared();
        Vec p;
        if (0 <= determinant && determinant <= 1) {
            p = bpos + p1 + determinant * u;
        }
        else if (determinant < 0) {
            p = bpos + p1;
        }
        else {
            p = bpos + p2;
        }
        auto ray = p - apos;
        if (ray.abs() < abs(d.distance) || !i) {
            if (u.perp().dot(ray) < 0) {
                d.distance = ray.abs();
                d.normal = ray.norm();
            }
            else {
                d.distance = -ray.abs();
                d.normal = -ray.norm();
            }
            d.a_point = apos + d.normal * a_circle.radius;
            d.b_point = p;
            d.distance -= a_circle.radius;
        }
    }
    return d;
}

DistanceResult distanceBetweenPolygonCircle(const Shape& p, const Body& pb, const Shape& c, const Body& cb) {
    auto d = distanceBetweenCirclePolygon(c, cb, p, pb);
    return {d.distance, d.b_point, d.a_point, -d.normal};
}

tuple<double, Vec, Vec> axis_proj(const vector<Vec>& points, Vec axis) {
    double min;
    bool initial = true;
    Vec p1, p2;
    for (const auto& point : points) {
        auto dot = point.dot(axis);
        if (initial || dot < min) {
            min = dot;
            p1 = p2 = point;
            initial = false;
        }
        else if (dot == min) {
            p2 = point;
        }
    }
    return {min, p1, p2};
}

tuple<double, Vec, Vec> axis_proj_poly(const Polygon& a_poly, const Polygon& b_poly, Vec ray) {
    Vec p1, p2, min_axis, axis;
    double first_min, second_min;

    for (unsigned int i = 0; i < a_poly.points.size(); ++i) {
        auto v1 = a_poly.points[i], v2 = a_poly.points[(i + 1) % a_poly.points.size()];
        axis = (v2 - v1).norm().perp();
        auto mins = axis_proj(b_poly.points, axis);
        auto min = get<0>(mins) - v1.dot(axis) + ray.dot(axis);
        if (!i || min > first_min) {
            second_min = first_min;
            first_min = min;
            min_axis = axis;

            if (i >= 1 && (get<1>(mins) == p2 || get<2>(mins) == p2)) {
                p1 = get<2>(mins); p2 = get<1>(mins);
            }
            else {
                p1 = get<1>(mins); p2 = get<2>(mins);
            }
        }
        else if (i >= 1 && min >= second_min) {
            second_min = min;
            if (get<1>(mins) == p2 || get<2>(mins) == p2) {
                p1 = p2;
            }
        }
    }
    return {first_min, min_axis, p1};
}

DistanceResult distanceBetweenPolygonPolygon(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_poly = dynamic_cast<const Polygon&>(a);
    const auto& b_poly = dynamic_cast<const Polygon&>(b);

    auto apos = a_body.position() + a_poly.position;
    auto bpos = b_body.position() + b_poly.position;
    auto ray = bpos - apos;
    DistanceResult dist;

    auto res = axis_proj_poly(a_poly, b_poly, ray);
    dist.distance = get<0>(res);
    dist.normal = get<1>(res);
    dist.b_point = get<2>(res) + bpos;
    dist.a_point = dist.b_point - dist.normal * dist.distance;

    res = axis_proj_poly(b_poly, a_poly, -ray);
    if (get<0>(res) > dist.distance) {
        dist.distance = get<0>(res);
        dist.normal = -get<1>(res);
        dist.a_point = get<2>(res) + apos;
        dist.b_point = dist.a_point + dist.normal * dist.distance;
    }
    return dist;
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
