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

#ifndef VEC_HPP
#define VEC_HPP

#include <cmath>
#include <iostream>
#include <tuple>

class Vec {
public:
    constexpr Vec(double x_=0, double y_=0) : x(x_), y(y_) {
    }

    inline Vec operator-() {
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

    inline void operator*=(const Vec& other) {
        x *= other.x;
        y *= other.y;
    }

    inline void operator/=(double factor) {
        x /= factor;
        y /= factor;
    }

    inline void operator/=(const Vec& other) {
        x /= other.x;
        y /= other.y;
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

    inline double abs() const {
        return std::hypot(x, y);
    }

    inline double bearing() const {
        return std::atan2(y, x);
    }

    inline double distance_to(const Vec& other) const {
        return std::hypot(other.x - x, other.y - y);
    }

    inline double bearing_to(const Vec& other) const {
        return std::atan2(other.y - y, other.x - x);
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

    inline static Vec fromBearing(double bearing) {
        return {std::cos(bearing), std::sin(bearing)};
    }

    double x, y;
};

inline Vec operator+(Vec a, const Vec& b) {
    a += b;
    return a;
}

inline Vec operator-(Vec a, const Vec& b) {
    a -= b;
    return a;
}

inline Vec operator*(Vec vec, double factor) {
    vec *= factor;
    return vec;
}

inline Vec operator*(double factor, Vec vec) {
    vec *= factor;
    return vec;
}

inline Vec operator*(Vec a, const Vec& b) {
    a *= b;
    return a;
}

inline Vec operator/(Vec vec, double factor) {
    vec /= factor;
    return vec;
}

inline Vec operator/(Vec a, const Vec& b) {
    a /= b;
    return a;
}

std::ostream& operator<<(std::ostream& os, const Vec& vec);

#endif // VEC_HPP
