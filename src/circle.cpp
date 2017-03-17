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

#include "circle.hpp"
#include "body.hpp"

Circle::Circle(double radius_/*=0*/, double mass_/*=0*/, const Vec& position_/*={}*/,
               double radar_cross_section/*=0*/, double radar_emissions/*=0*/, double thermal_emissions/*=0*/) : Shape(mass_,
                                                                                                                       position_,
                                                                                                                       radar_cross_section,
                                                                                                                       radar_emissions,
                                                                                                                       thermal_emissions),
                                                                                                                 radius(radius_) {
}

AABB Circle::aabb() const {
    return {-radius, radius, -radius, radius};
}

Shape* Circle::clone() const {
    return new Circle{radius, mass, position, signature.radar_cross_section, signature.radar_emissions, signature.thermal_emissions};
}

bool Circle::canCollide() const {
    return true;
}

CollisionTimeResult Circle::collide(const Shape* other, double end_time, bool entering) const {
    return other->collide(this, end_time, entering);
}

CollisionTimeResult Circle::collide(const Circle* other, double end_time, bool entering) const {
    return collideCircleCircle(this, other, end_time, entering);
}

CollisionTimeResult Circle::collide(const Polygon* other, double end_time, bool entering) const {
    return collideCirclePolygon(this, other, end_time, entering);
}

// LCOV_EXCL_START
CollisionTimeResult Circle::collide(const MassShape* /*other*/, double /*end_time*/, bool /*entering*/) const {
    return {};
}
// LCOV_EXCL_STOP


bool Circle::immediate_collide(const Shape* other) const {
    return other->immediate_collide(this);
}

bool Circle::immediate_collide(const Circle* other) const {
    return (other->body->position() - body->position()).abs() < other->radius + radius;
}

bool Circle::immediate_collide(const Polygon* other) const {
    return immediateCollideCirclePolygon(this, other);
}

// LCOV_EXCL_START
bool Circle::immediate_collide(const MassShape* /*other*/) const {
    return false;
}
// LCOV_EXCL_STOP
