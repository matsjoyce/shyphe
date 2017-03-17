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
class MassShape;
class Polygon;

struct Signature {
    double radar_emissions = 0;
    double thermal_emissions = 0;
    double radar_cross_section = 0;

    constexpr Signature(double re=0, double te=0, double rcs=0) : radar_emissions(re), thermal_emissions(te), radar_cross_section(rcs) {
    }

    operator bool() const {
        return radar_emissions || thermal_emissions || radar_cross_section;
    }

    void operator&=(const Signature& other) {
        radar_emissions = std::max(radar_emissions, other.radar_emissions);
        thermal_emissions = std::max(thermal_emissions, other.thermal_emissions);
        radar_cross_section = std::max(radar_cross_section, other.radar_cross_section);
    }

    bool approx_equals(const Signature& other, double ratio) const {
        return (
            (other.radar_emissions >= radar_emissions * ratio) && (other.radar_emissions <= radar_emissions / ratio)
            && (other.thermal_emissions >= thermal_emissions * ratio) && (other.thermal_emissions <= thermal_emissions / ratio)
            && (other.radar_cross_section >= radar_cross_section * ratio) && (other.radar_cross_section <= radar_cross_section / ratio)
        );
    }
};

class Shape {
public:
    Body* body = nullptr;
    double mass = 0;
    Vec position;
    Signature signature;

    Shape(double mass_=0, const Vec& position_={}, double radar_cross_section=0, double radar_emissions=0, double thermal_emissions=0);
    virtual ~Shape();
    virtual AABB aabb() const = 0;
    virtual Shape* clone() const = 0;
    virtual bool canCollide() const = 0;

    // Double dispatch
    virtual CollisionTimeResult collide(const Shape* other, double end_time, bool entering) const = 0;
    virtual CollisionTimeResult collide(const Circle* other, double end_time, bool entering) const = 0;
    virtual CollisionTimeResult collide(const MassShape* other, double end_time, bool entering) const = 0;
    virtual CollisionTimeResult collide(const Polygon* other, double end_time, bool entering) const = 0;

    // Double dispatch
    virtual bool immediate_collide(const Shape* other) const = 0;
    virtual bool immediate_collide(const Circle* other) const = 0;
    virtual bool immediate_collide(const MassShape* other) const = 0;
    virtual bool immediate_collide(const Polygon* other) const = 0;
};

#endif // SHAPE_HPP
