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

Circle::Circle(double radius_/*=0*/, double mass_/*=0*/, const Vec& position_/*={}*/) : Shape(mass_, position_), radius(radius_) {
}

AABB Circle::aabb() const {
    return {-radius, radius, -radius, radius};
}

Shape* Circle::clone() const {
    return new Circle{radius, mass, position};
}

CollisionTimeResult Circle::collide(const Shape* other, double end_time) const {
    return other->collide(this, end_time);
}

CollisionTimeResult Circle::collide(const Circle* other, double end_time) const {
    return collideCircleCircle(this, other, end_time);
}

// CollisionTimeResult Circle::collide(const Polygon* other) const {
// }
