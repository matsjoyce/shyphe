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

CollisionTimeResult collideShapes(Shape* a, Shape* b, double end_time, bool ignore_initial) {
    DistanceDispatch dist_func = DISPATCH_TABLE.at(make_pair(a->shape_type(), b->shape_type()));
    Body abody = *a->body, bbody = *b->body;
    auto vel_diff = abody.velocity() - bbody.velocity();
    DistanceResult current_distance;
    double time = 0;
    unsigned int iteration = 0;
    while (iteration < 1000) {
        current_distance = dist_func(*a, abody, *b, bbody);

        double vel = vel_diff.dot(current_distance.normal)
                     + (a->position.abs() + a->boundingRadius()) * abs(a->body->angularVelocity())
                     + (b->position.abs() + b->boundingRadius()) * abs(b->body->angularVelocity());
        double add_time = abs(current_distance.distance) / vel;
        time += add_time;

        abody.updatePosition(add_time);
        bbody.updatePosition(add_time);

        if (time > end_time) {
            return {};
        }
        if (current_distance.distance < COLLISION_LIMIT) {
            auto vel_at = vel_diff
                          - (current_distance.a_point - abody.position()).perp() * a->body->angularVelocity()
                          + (current_distance.b_point - bbody.position()).perp() * b->body->angularVelocity();
            if (vel_at.dot(current_distance.normal) > 0 && !ignore_initial) {
                break;
            }
            add_time = max(COLLISION_LIMIT * 3, COLLISION_LIMIT * 3 / abs(vel_at.dot(current_distance.normal)));
            time += add_time;
            abody.updatePosition(add_time);
            bbody.updatePosition(add_time);
        }
        else {
            ignore_initial = false;
        }
        if (vel <= 0 || time < 0) {
            return {};
        }

        ++iteration;
    }
    return {a->body, b->body, a, b, time, (current_distance.a_point + current_distance.b_point) / 2.0, current_distance.normal};
}

DistanceResult distanceBetweenCircleCircle(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_circle = dynamic_cast<const Circle&>(b);

    auto apos = a_body.position() + a_circle.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_circle.position.rotate(b_body.angle());
    auto ray = bpos - apos;
    auto norm = ray ? ray.norm() : Vec{1, 0};
    return {ray.abs() - a_circle.radius - b_circle.radius, apos + a_circle.radius * norm, bpos - b_circle.radius * norm, norm};
}

DistanceResult distanceBetweenCirclePolygon(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_poly = dynamic_cast<const Polygon&>(b);

    auto apos = a_body.position() + a_circle.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_poly.position.rotate(b_body.angle());
    cout << a_circle.position << bpos << endl;
    DistanceResult d;
    for (unsigned int i = 0; i < b_poly.points.size(); ++i) {
        auto p1 = b_poly.points[i].rotate(b_body.angle()), p2 = b_poly.points[(i + 1) % b_poly.points.size()].rotate(b_body.angle());
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

tuple<double, Vec, Vec> axis_proj(const vector<Vec>& points, Vec axis, double angle) {
    double min;
    bool initial = true;
    Vec p1, p2;
    for (const auto& point : points) {
        auto rpoint = point.rotate(angle);
        auto dot = rpoint.dot(axis);
        if (initial || dot < min) {
            min = dot;
            p1 = p2 = rpoint;
            initial = false;
        }
        else if (dot == min) {
            p2 = rpoint;
        }
    }
    return {min, p1, p2};
}

tuple<double, Vec, Vec> axis_proj_poly(const Polygon& a_poly, const Polygon& b_poly, Vec ray, double a_angle, double b_angle) {
    Vec p1, p2, min_axis, axis;
    double first_min, second_min;

    for (unsigned int i = 0; i < a_poly.points.size(); ++i) {
        auto v1 = a_poly.points[i].rotate(a_angle), v2 = a_poly.points[(i + 1) % a_poly.points.size()].rotate(a_angle);
        axis = (v2 - v1).norm().perp();
        auto mins = axis_proj(b_poly.points, axis, b_angle);
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

    auto apos = a_body.position() + a_poly.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_poly.position.rotate(b_body.angle());
    auto ray = bpos - apos;
    DistanceResult dist;

    auto res = axis_proj_poly(a_poly, b_poly, ray, a_body.angle(), b_body.angle());
    dist.distance = get<0>(res);
    dist.normal = get<1>(res);
    dist.b_point = get<2>(res) + bpos;
    dist.a_point = dist.b_point - dist.normal * dist.distance;

    res = axis_proj_poly(b_poly, a_poly, -ray, b_body.angle(), a_body.angle());
    if (get<0>(res) > dist.distance) {
        dist.distance = get<0>(res);
        dist.normal = -get<1>(res);
        dist.a_point = get<2>(res) + apos;
        dist.b_point = dist.a_point + dist.normal * dist.distance;
    }
    return dist;
}

inline double square(double x) {
    return x * x;
}

CollisionResult collisionResult(const CollisionTimeResult& cr, const CollisionParameters& params) {
    const Body& a = *cr.a;
    const Body& b = *cr.b;
    Vec a_perp_touch_point = -(cr.touch_point - a.position()).perp();
    Vec b_perp_touch_point = -(cr.touch_point - b.position()).perp();
    Vec a_vel = a.velocity() + a_perp_touch_point * a.angularVelocity();
    Vec b_vel = b.velocity() + b_perp_touch_point * b.angularVelocity();
    double relative_vel = (b_vel - a_vel).dot(cr.normal);
    if (relative_vel >= 0) {
        throw runtime_error("Collision with no movement");
    }
    double top = (1 + params.restitution) * relative_vel;
    double bottom = 1 / a.mass() + square(a_perp_touch_point.dot(cr.normal)) / a.momentOfInertia()
                  + 1 / b.mass() + square(b_perp_touch_point.dot(cr.normal)) / b.momentOfInertia();
    return {cr.normal * top / bottom, cr.normal * relative_vel};
}
