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

#ifndef SHYPHE_AABB_HPP
#define SHYPHE_AABB_HPP

#include "vec.hpp"

namespace shyphe {
    class AABB {
    public:
        double min_x, max_x, min_y, max_y;

        AABB(double min_x_, double max_x_, double min_y_, double max_y_) : min_x(min_x_), max_x(max_x_),
                                                                           min_y(min_y_), max_y(max_y_) {

        }

        AABB(const Vec& center, double width, double height) : min_x(center.x - width / 2), max_x(center.x + width / 2),
                                                               min_y(center.y - height / 2), max_y(center.y + height / 2) {

        }

        AABB(const Vec& corner, const Vec& opposite_corner) : min_x(std::min(corner.x, opposite_corner.x)),
                                                              max_x(std::max(corner.x, opposite_corner.x)),
                                                              min_y(std::min(corner.y, opposite_corner.y)),
                                                              max_y(std::max(corner.y, opposite_corner.y)) {

        }

        inline void operator&=(const AABB& other) {
            min_x = std::min(min_x, other.min_x);
            max_x = std::max(max_x, other.max_x);
            min_y = std::min(min_y, other.min_y);
            max_y = std::max(max_y, other.max_y);
        }

        inline void operator+=(const Vec& other) {
            min_x += other.x;
            max_x += other.x;
            min_y += other.y;
            max_y += other.y;
        }

        inline Vec bottomleft() const {
            return {min_x, min_y};
        }

        inline Vec bottomright() const {
            return {max_x, min_y};
        }

        inline Vec topleft() const {
            return {min_x, max_y};
        }

        inline Vec topright() const {
            return {max_x, max_y};
        }

        inline Vec center() const {
            return {(min_x + max_x) / 2.0, (min_y + max_y) / 2.0};
        }
    };

    inline AABB operator&(AABB a, const AABB& b) {
        a &= b;
        return a;
    }

    inline AABB operator+(AABB a, const Vec& b) {
        a += b;
        return a;
    }

    inline AABB operator+(const Vec& b, AABB a) {
        a += b;
        return a;
    }

    std::ostream& operator<<(std::ostream& os, const AABB& bb);
}

#endif // SHYPHE_AABB_HPP
