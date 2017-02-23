/*
 * LunyFringe.core.engine.physics - rigid body physics engine
 * Copyright (C) 2016 Matthew Joyce matsjoyce@gmail.com
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

#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>

constexpr double pi() {
    return 3.141592653589793238462643383279502884;
}

constexpr double dpi() {
    // Double pi
    return 2 * 3.141592653589793238462643383279502884;
}

constexpr double deg_to_rad() {
    return pi() / 180.0;
}

constexpr double rad_to_deg() {
    return 180.0 / pi();
}

constexpr inline double norm_rad(double angle) {
    return std::remainder(angle, dpi());
}

constexpr inline double norm_deg(double angle) {
    return std::remainder(angle, 360.0);
}

constexpr inline double to_deg(double angle) {
    return norm_deg(angle * rad_to_deg());
}

constexpr inline double to_rad(double angle) {
    return norm_rad(angle * deg_to_rad());
}

#endif // UTILS_HPP
