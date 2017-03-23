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

#include "circle.hpp"
#include "body.hpp"

using namespace std;

Circle::Circle(double radius_/*=0*/, double mass_/*=0*/, const Vec& position_/*={}*/,
               double radar_cross_section/*=0*/, double radar_emissions/*=0*/, double thermal_emissions/*=0*/) : Shape(mass_,
                                                                                                                       position_,
                                                                                                                       radar_cross_section,
                                                                                                                       radar_emissions,
                                                                                                                       thermal_emissions),
                                                                                                                 radius(radius_) {
}

AABB Circle::aabb(double /*angle*/) const
{
    return {-radius, radius, -radius, radius};
}

Shape* Circle::clone() const {
    return new Circle{radius, mass, position, signature.radar_cross_section, signature.radar_emissions, signature.thermal_emissions};
}

bool Circle::canCollide() const {
    return true;
}

type_index Circle::shape_type() const {
    return {typeid(Circle)};
}

double Circle::boundingRadius() const {
    return radius;
}

double Circle::momentOfInertia() const {
    return mass * radius * radius / 2;
}
