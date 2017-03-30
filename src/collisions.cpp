/*
 * shyphe - Stiff HIgh velocity PHysics Engine
 * Copyright (C) 2017 Matthew Joyce matsjoyce@gmail.com
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
using namespace shyphe;

const double COLLISION_LIMIT = 1e-8;
const unsigned int MAX_ITERATIONS = 1000;
typedef DistanceResult (*DistanceDispatch)(const Shape&, const Body&, const Shape&, const Body&);

const map<pair<type_index, type_index>, DistanceDispatch> DISPATCH_TABLE = {
    {{type_index(typeid(Circle)), type_index(typeid(Circle))}, &distanceBetweenCircleCircle},
    {{type_index(typeid(Circle)), type_index(typeid(Polygon))}, &distanceBetweenCirclePolygon},
    {{type_index(typeid(Polygon)), type_index(typeid(Circle))}, &distanceBetweenPolygonCircle},
    {{type_index(typeid(Polygon)), type_index(typeid(Polygon))}, &distanceBetweenPolygonPolygon}
};

DistanceResult shyphe::distanceBetween(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    DistanceDispatch dist_func = DISPATCH_TABLE.at(make_pair(a.shape_type(), b.shape_type()));
    return dist_func(a, a_body, b, b_body);
}

CollisionTimeResult shyphe::collideShapes(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body, double end_time, bool ignore_initial) {
    // Based on algorithm from bottom of http://www.wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/
    DistanceDispatch dist_func = DISPATCH_TABLE.at(make_pair(a.shape_type(), b.shape_type()));
    Body abody = a_body, bbody = b_body;
    auto vel_diff = abody.velocity() - bbody.velocity();
    DistanceResult current_distance;
    double time = 0;
    unsigned int iteration = 0;
    while (iteration < MAX_ITERATIONS) {
        current_distance = dist_func(a, abody, b, bbody);
        double add_time = 0;

        if (current_distance.distance < COLLISION_LIMIT) {
            auto vel_at = vel_diff
                          - (current_distance.a_point - abody.position()).perp() * abody.angularVelocity()
                          + (current_distance.b_point - bbody.position()).perp() * bbody.angularVelocity();
            if (vel_at.dot(current_distance.normal) > COLLISION_LIMIT && !ignore_initial) {
                return {time, (current_distance.a_point + current_distance.b_point) / 2.0, current_distance.normal};
            }
            add_time += max(COLLISION_LIMIT * 3, COLLISION_LIMIT * 3 / abs(vel_at.dot(current_distance.normal)));
        }
        else {
            ignore_initial = false;
        }

        double vel = vel_diff.dot(current_distance.normal)
                     + (a.position.abs() + a.boundingRadius()) * abs(abody.angularVelocity())
                     + (b.position.abs() + b.boundingRadius()) * abs(bbody.angularVelocity());

        if (vel <= 0) {
            return {};
        }

        add_time += abs(current_distance.distance) / vel;
        time += add_time;

        if (time < 0 || time > end_time) {
            return {};
        }

        abody.updatePosition(add_time);
        bbody.updatePosition(add_time);
        ++iteration;
    }
    return {};
}

DistanceResult shyphe::distanceBetweenCircleCircle(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_circle = dynamic_cast<const Circle&>(b);

    auto apos = a_body.position() + a_circle.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_circle.position.rotate(b_body.angle());
    auto ray = bpos - apos;
    auto norm = ray ? ray.norm() : Vec{1, 0};
    return {ray.abs() - a_circle.radius - b_circle.radius, apos + a_circle.radius * norm, bpos - b_circle.radius * norm, norm};
}

Vec closestPointOnLineseg(Vec point, Vec l1, Vec l2) {
    auto u = l2 - l1, v = point - l1;
    auto determinant = u.dot(v) / u.squared();
    Vec p;
    if (0 <= determinant && determinant <= 1) {
        p = l1 + determinant * u;
    }
    else if (determinant < 0) {
        p = l1;
    }
    else {
        p = l2;
    }
    return p;
}

int updateMinimumDistance(DistanceResult& dist, Vec point, Vec l1, Vec l2, Vec line_pos, int number, bool initial, bool invert) {
    auto p = closestPointOnLineseg(point - line_pos, l1, l2) + line_pos;
    auto ray = p - point;

    if (ray.abs() < abs(dist.distance) || initial) {
        dist.normal = ray ? ray.norm() : -(l2 - l1).perp().norm();
        dist.distance = ray.abs();
        if ((l2 - l1).cross(dist.normal) > 0) {
            dist.distance = -dist.distance;
            dist.normal = -dist.normal;
        }
        dist.a_point = point;
        dist.b_point = p;
        if (invert) {
            swap(dist.a_point, dist.b_point);
            dist.normal = -dist.normal;
        }
        number = 1;
    }
    else if (number && (((l2 - l1).cross(ray) > 0) ? -ray.abs() : ray.abs()) == dist.distance) {
        if (invert) {
            dist.a_point += p;
            dist.b_point += point;
        }
        else {
            dist.a_point += point;
            dist.b_point += p;
        }
        ++number;
    }
    return number;
}

DistanceResult shyphe::distanceBetweenCirclePolygon(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_circle = dynamic_cast<const Circle&>(a);
    const auto& b_poly = dynamic_cast<const Polygon&>(b);

    auto apos = a_body.position() + a_circle.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_poly.position.rotate(b_body.angle());
    DistanceResult d;

    for (unsigned int i = 0; i < b_poly.points.size(); ++i) {
        auto p1 = b_poly.points[i].rotate(b_body.angle()), p2 = b_poly.points[(i + 1) % b_poly.points.size()].rotate(b_body.angle());
        updateMinimumDistance(d, apos, p1, p2, bpos, 0, !i, false);
    }
    d.a_point += d.normal * a_circle.radius;
    d.distance -= a_circle.radius;
    return d;
}

DistanceResult shyphe::distanceBetweenPolygonCircle(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    auto d = distanceBetweenCirclePolygon(b, b_body, a, a_body);
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

tuple<double, Vec, Vec, Vec, Vec> axis_proj_poly(const Polygon& a_poly, const Polygon& b_poly, Vec ray, double a_angle, double b_angle) {
    tuple<double, Vec, Vec, Vec, Vec> res;

    for (unsigned int i = 0; i < a_poly.points.size(); ++i) {
        auto v1 = a_poly.points[i].rotate(a_angle), v2 = a_poly.points[(i + 1) % a_poly.points.size()].rotate(a_angle);
        auto axis = (v2 - v1).norm().perp();
        auto mins = axis_proj(b_poly.points, axis, b_angle);
        auto min = get<0>(mins) - v1.dot(axis) + ray.dot(axis);
        if (!i || min > get<0>(res)) {
            get<0>(res) = min;
            get<1>(res) = v1;
            get<2>(res) = v2;
            get<3>(res) = get<1>(mins);
            get<4>(res) = get<2>(mins);
        }
    }
    return res;
}

DistanceResult shyphe::distanceBetweenPolygonPolygon(const Shape& a, const Body& a_body, const Shape& b, const Body& b_body) {
    const auto& a_poly = dynamic_cast<const Polygon&>(a);
    const auto& b_poly = dynamic_cast<const Polygon&>(b);

    auto apos = a_body.position() + a_poly.position.rotate(a_body.angle());
    auto bpos = b_body.position() + b_poly.position.rotate(b_body.angle());
    auto ray = bpos - apos;
    DistanceResult dist;

    // Use SAT to find closest edges, then find closest distance
    auto a_res = axis_proj_poly(a_poly, b_poly, ray, a_body.angle(), b_body.angle());
    auto b_res = axis_proj_poly(b_poly, a_poly, -ray, b_body.angle(), a_body.angle());
    Vec a1, a2, b1, b2, plane_norm;
    double dummy;

    if (get<0>(a_res) > get<0>(b_res)) {
        tie(dummy, a1, a2, b1, b2) = a_res;
    }
    else {
        tie(dummy, b1, b2, a1, a2) = b_res;
    }

    if (a1 == a2) {
        updateMinimumDistance(dist, apos + a1, b1, b2, bpos, 0, true, false);
    }
    else if (b1 == b2) {
        updateMinimumDistance(dist, bpos + b1, a1, a2, apos, 0, true, true);
    }
    else {
        auto n = updateMinimumDistance(dist, apos + a1, b1, b2, bpos, 0, true, false);
        n = updateMinimumDistance(dist, apos + a2, b1, b2, bpos, n, false, false);
        n = updateMinimumDistance(dist, bpos + b1, a1, a2, apos, n, false, true);
        n = updateMinimumDistance(dist, bpos + b2, a1, a2, apos, n, false, true);
        dist.a_point /= n;
        dist.b_point /= n;
    }
    return dist;
}

inline double square(double x) {
    return x * x;
}

CollisionResult shyphe::collisionResult(const CollisionTimeResult& cr, const Body& a, const Body& b, const CollisionParameters& params) {
    // http://chrishecker.com/images/e/e7/Gdmphys3.pdf
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
