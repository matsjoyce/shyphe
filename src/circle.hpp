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

#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "shape.hpp"
#include "collisions.hpp"

class Circle : public Shape {
public:
    double radius = 0;

    Circle(double radius_=0, double mass_=0, const Vec& position_={},
           double radar_cross_section=0, double radar_emissions=0, double thermal_emissions=0);
    virtual AABB aabb() const override;
    virtual Shape* clone() const override;
    virtual bool canCollide() const override;

    // Double dispatch
    virtual CollisionTimeResult collide(const Shape* other, double end_time, bool entering) const override;
    virtual CollisionTimeResult collide(const Circle* other, double end_time, bool entering) const override;
    virtual CollisionTimeResult collide(const MassShape* other, double end_time, bool entering) const override;
//     virtual CollisionTimeResult collide(const Polygon* other) const override;

    virtual bool immediate_collide(const Shape* other) const override;
    virtual bool immediate_collide(const Circle* other) const override;
    virtual bool immediate_collide(const MassShape* other) const override;
//     virtual bool immediate_collide(const Polygon* other) const override;
};

#endif // CIRCLE_HPP
