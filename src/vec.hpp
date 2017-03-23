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

#ifndef SHYPHE_VEC_HPP
#define SHYPHE_VEC_HPP

#include <cmath>
#include <iostream>
#include <tuple>

namespace shyphe {
    class Vec {
    public:
        constexpr Vec(double x_, double y_) : x(x_), y(y_) {
        }

        constexpr Vec() : x(0), y(0) {
        }

        inline Vec operator-() const {
            return {-x, -y};
        }

        inline void operator+=(const Vec& other) {
            x += other.x;
            y += other.y;
        }

        inline void operator-=(const Vec& other) {
            x -= other.x;
            y -= other.y;
        }

        inline void operator*=(double factor) {
            x *= factor;
            y *= factor;
        }

        inline void operator/=(double factor) {
            x /= factor;
            y /= factor;
        }

        inline bool operator==(const Vec& other) const {
            return std::tie(x, y) == std::tie(other.x, other.y);
        }

        inline bool operator!=(const Vec& other) const {
            return std::tie(x, y) != std::tie(other.x, other.y);
        }

        inline bool operator<(const Vec& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }

        inline bool operator<=(const Vec& other) const {
            return std::tie(x, y) <= std::tie(other.x, other.y);
        }

        inline bool operator>(const Vec& other) const {
            return std::tie(x, y) > std::tie(other.x, other.y);
        }

        inline bool operator>=(const Vec& other) const {
            return std::tie(x, y) >= std::tie(other.x, other.y);
        }

        explicit inline operator bool() const {
            return x || y;
        }

        inline double abs() const {
            return std::hypot(x, y);
        }

        inline double bearing() const {
            return std::atan2(x, y);
        }

        inline double distanceTo(const Vec& other) const {
            return std::hypot(other.x - x, other.y - y);
        }

        inline double bearingTo(const Vec& other) const {
            return std::atan2(other.x - x, other.y - y);
        }

        inline double cross(const Vec& other) const {
            return x * other.y - y * other.x;
        }

        inline double dot(const Vec& other) const {
            return x * other.x + y * other.y;
        }

        inline double squared() const {
            return x * x + y * y;
        }

        inline Vec norm() const {
            auto mag = abs();
            return {x / mag, y / mag};
        }

        inline Vec perp() const {
            return {-y, x};
        }

        inline Vec reflect(const Vec& axis) const {
            Vec res = proj(axis);
            res *= 2;
            res -= *this;
            return res;
        }

        inline Vec proj(const Vec& axis) const {
            Vec res = axis;
            res *= dot(axis) / axis.squared();
            return res;
        }

        inline Vec rej(const Vec& axis) const {
            Vec res = axis.perp();
            res *= dot(res) / axis.squared();
            return res;
        }

        inline static Vec fromBearing(double bearing) {
            return {std::sin(bearing), std::cos(bearing)};
        }

        inline Vec rotate(double bearing) const {
            // Clockwise
            double c = std::cos(bearing), s = std::sin(bearing);
            return {c * x + s * y, -s * x + c * y};
        }

        double x, y;
    };

    inline Vec operator+(const Vec& a, const Vec& b) {
        Vec res = a;
        res += b;
        return res;
    }

    inline Vec operator-(const Vec& a, const Vec& b) {
        Vec res = a;
        res -= b;
        return res;
    }

    inline Vec operator*(const Vec& vec, double factor) {
        Vec res = vec;
        res *= factor;
        return res;
    }

    inline Vec operator*(double factor, const Vec& vec) {
        Vec res = vec;
        res *= factor;
        return res;
    }

    inline Vec operator/(const Vec& vec, double factor) {
        Vec res = vec;
        res /= factor;
        return res;
    }

    std::ostream& operator<<(std::ostream& os, const Vec& vec);
}

#endif // SHYPHE_VEC_HPP
