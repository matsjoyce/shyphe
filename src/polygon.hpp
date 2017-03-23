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

#ifndef SHYPHE_POLYGON_HPP
#define SHYPHE_POLYGON_HPP

#include <vector>
#include "shape.hpp"
#include "vec.hpp"

namespace shyphe {
    class Polygon : public Shape {
    public:
        std::vector<Vec> points;

        Polygon(const std::vector<Vec>& points_={}, double mass_=0, const Vec& position_={},
                double radar_cross_section=0, double radar_emissions=0, double thermal_emissions=0);

        virtual AABB aabb(double angle) const override;
        virtual Shape* clone() const override;
        virtual bool canCollide() const override;
        virtual std::type_index shape_type() const override;
        virtual double boundingRadius() const override;
        virtual double momentOfInertia() const override;
    };
}

#endif // SHYPHE_POLYGON_HPP
