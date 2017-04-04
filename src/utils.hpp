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

#ifndef SHYPHE_UTILS_HPP
#define SHYPHE_UTILS_HPP

#include <cmath>

namespace shyphe {
    constexpr const char* version() {
        return "0.1.0-alpha";
    }

    constexpr unsigned int hexversion() {
        return 0x000100;
    }

    struct Metadata {
        const char* version = shyphe::version();
        unsigned int hexversion = shyphe::hexversion();
        const char* author = "Matthew Joyce";
        const char* author_email = "matsjoyce@gmail.com";
        const char* copyright = "2017, Matthew Joyce";
        const char* credits = "Matthew Joyce";
        const char* license = "LGPLv3";
        const char* maintainer = "Matthew Joyce";
        const char* email = "matsjoyce@gmail.com";
        const char* status = "Development";
    };

    constexpr Metadata metadata() {
        return {};
    }

    constexpr double pi() {
        return 3.141592653589793238462643383279502884;
    }

    constexpr double hpi() {
        // Half pi
        return 3.141592653589793238462643383279502884 / 2;
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
        // radians are in the range [-pi, pi]
        return std::remainder(angle, dpi());
    }

    constexpr inline double norm_deg(double angle) {
        // degrees are in the range (0, 360]
        auto x = std::remainder(angle, 360.0);
        return x < 0 ? x + 360.0 : x;
    }

    constexpr inline double to_deg(double angle) {
        return norm_deg(angle * rad_to_deg());
    }

    constexpr inline double to_rad(double angle) {
        return norm_rad(angle * deg_to_rad());
    }

    constexpr inline double angle_diff_rad(double a, double b) {
        double d = norm_rad(a - b);
        if (d > pi()) {
            d -= dpi();
        }
        return d;
    }

    constexpr inline double angle_diff_deg(double a, double b) {
        double d = norm_deg(a - b);
        if (d > 180.0) {
            d -= 360.0;
        }
        return d;
    }
}

#endif // SHYPHE_UTILS_HPP
