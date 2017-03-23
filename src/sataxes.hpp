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

#ifndef SATAXES_HPP
#define SATAXES_HPP

#include <vector>
#include <set>
#include <utility>
#include "body.hpp"

struct SATShadow {
    double position;
    bool start;
    Body* body;
};

class SATAxes{
public:
    void addBody(Body* body, double time);
    void removeBody(Body* body);
    void reset(int reserve_hint=0);
    std::set<std::pair<Body*, Body*>> possibleCollisions();
private:
    std::vector<SATShadow> x_axis, y_axis;

    std::set<std::pair<Body*, Body*>> _collisionsOnAxis(const std::vector<SATShadow>& axis);
};

#endif // SATAXES_HPP
