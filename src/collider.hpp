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

#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include <vector>
#include <utility>
#include <map>
#include <tuple>
#include <set>

#include "body.hpp"
#include "vec.hpp"
#include "sataxes.hpp"
#include "collisions.hpp"

struct Collision {
    Body* body;
    Body* other;
    double time;
    Vec impulse;
};

class Collider {
public:
    void newBody(Body* body);
    void reset(double time);
    std::pair<Collision, Collision> nextCollision();
    void finishedCollision();
    bool hasNextCollision();
private:
    double time_until, current_time;
    std::vector<Body*> bodies;
    std::map<Body*, double> body_times;
    std::set<Body*> changed_bodies;
    std::vector<CollisionTimeResult> collision_times;
    SATAxes sat_axes;

    void _updateCollisionTimes();
    void _updateCollisionTimesChanged();
};

#endif // COLLIDER_HPP
