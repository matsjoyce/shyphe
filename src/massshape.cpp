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

using namespace std;

Shape* MassShape::clone() const {
    return new MassShape(mass, position, signature.radar_cross_section, signature.radar_emissions, signature.thermal_emissions);
}

bool MassShape::canCollide() const {
    return false;
}

double MassShape::momentOfInertia() const {
    return 1;
}

// LCOV_EXCL_START
type_index MassShape::shape_type() const {
    return {typeid(MassShape)};
}

double MassShape::boundingRadius() const {
    return 0;
}

AABB MassShape::aabb(double /*angle*/) const {
    return {0, 0, 0, 0};
}
// LCOV_EXCL_STOP
