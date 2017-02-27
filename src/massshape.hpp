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

#ifndef MASSSHAPE_HPP
#define MASSSHAPE_HPP

#include "shape.hpp"

class MassShape : public Shape {
public:
    MassShape(double mass_=0);

    virtual AABB aabb() const override;
    virtual Shape* clone() const override;
    virtual bool canCollide() const override;

    virtual CollisionTimeResult collide(const Shape* other, double end_time) const override;
    virtual CollisionTimeResult collide(const Circle* other, double end_time) const override;
    virtual CollisionTimeResult collide(const MassShape* other, double end_time) const override;

    virtual bool immediate_collide(const Shape* other) const override;
    virtual bool immediate_collide(const Circle* other) const override;
    virtual bool immediate_collide(const MassShape* other) const override;
};

#endif // MASSSHAPE_HPP