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

#include "massshape.hpp"

MassShape::MassShape(double mass_/*=0*/, const Vec& position_/*={}*/,
                     double radar_cross_section/*=0*/, double radar_emissions/*=0*/, double thermal_emissions/*=0*/) : Shape(mass_,
                                                                                                                             position_,
                                                                                                                             radar_cross_section,
                                                                                                                             radar_emissions,
                                                                                                                             thermal_emissions) {
}

Shape* MassShape::clone() const {
    return new MassShape(mass);
}

bool MassShape::canCollide() const {
    return false;
}

// LCOV_EXCL_START
AABB MassShape::aabb() const {
    return {0, 0, 0, 0};
}

CollisionTimeResult MassShape::collide(const Shape* /*other*/, double /*end_time*/, bool /*entering*/) const {
    return CollisionTimeResult{};
}

CollisionTimeResult MassShape::collide(const Circle* /*other*/, double /*end_time*/, bool /*entering*/) const {
    return CollisionTimeResult{};
}

CollisionTimeResult MassShape::collide(const MassShape* /*other*/, double /*end_time*/, bool /*entering*/) const {
    return CollisionTimeResult{};
}

bool MassShape::immediate_collide(const Shape* /*other*/) const {
    return false;
}

bool MassShape::immediate_collide(const Circle* /*other*/) const {
    return false;
}

bool MassShape::immediate_collide(const MassShape* /*other*/) const {
    return false;
}
// LCOV_EXCL_STOP
