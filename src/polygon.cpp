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

#include "polygon.hpp"
#include <algorithm>

using namespace std;

Polygon::Polygon(const std::vector<Vec>& points_/*={}*/, double mass_/*=0*/, const Vec& position_/*={}*/,
                 double radar_cross_section/*=0*/, double radar_emissions/*=0*/, double thermal_emissions/*=0*/) : Shape(mass_, position_,
                                                                                                                         radar_cross_section,
                                                                                                                         radar_emissions,
                                                                                                                         thermal_emissions),
                                                                                                                    points(points_) {
    // http://stackoverflow.com/a/1881201/3946766
    if (points.size() >= 3) {
        double example = 0;
        for (unsigned int i = 0; i < points.size(); ++i) {
            auto p1 = points[i], p2 = points[(i + 1) % points.size()], p3 = points[(i + 2) % points.size()];
            auto cross = (p2 - p1).cross(p3 - p2);
            if (cross == 0) {
                points.erase(points.begin() + (i + 1) % points.size());
                --i;
            }
            else if (!example) {
                example = cross;
            }
            else if ((example > 0 && cross < 0) || (example < 0 && cross > 0)) {
                throw runtime_error("Polygon is concave, this is not supported");
            }
        }
        if (example > 0) {
            reverse(points.begin(), points.end());
        }
    }
    else {
        throw runtime_error("Not enough points");
    }
}

AABB Polygon::aabb(double angle) const {
    double minx, maxx, miny, maxy;
    bool initial = true;

    for (const auto& point : points) {
        auto rpoint = point.rotate(angle);
        if (initial) {
            minx = maxx = rpoint.x;
        }
        else if (rpoint.x < minx) {
            minx = rpoint.x;
        }
        else if (rpoint.x > maxx) {
            maxx = rpoint.x;
        }
        if (initial) {
            miny = maxy = rpoint.y;
        }
        else if (rpoint.y < miny) {
            miny = rpoint.y;
        }
        else if (rpoint.y > maxy) {
            maxy = rpoint.y;
        }
        initial = false;
    }
    return {minx, maxx, miny, maxy};
}

Shape* Polygon::clone() const {
    return new Polygon(points, mass, position, signature.radar_cross_section, signature.radar_emissions, signature.thermal_emissions);
}

bool Polygon::canCollide() const {
    return true;
}

type_index Polygon::shape_type() const {
    return {typeid(Polygon)};
}

double Polygon::boundingRadius() const {
    double br = 0;
    for (const auto& point : points) {
        br = max(br, point.abs());
    }
    return br;
}

double Polygon::momentOfInertia() const {
    double top = 0, bottom = 0;
    for (unsigned int i = 0; i < points.size(); ++i) {
        auto p1 = points[i], p2 = points[(i + 1) % points.size()];
        auto c = p2.cross(p1);
        bottom += c;
        top += c * (p1.squared() + p1.dot(p2) + p2.squared());
    }
    return top / bottom * mass / 6;
}
