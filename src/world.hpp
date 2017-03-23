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

#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include <vector>
#include <utility>
#include <map>
#include <set>
#include <memory>

#include "body.hpp"
#include "vec.hpp"
#include "sataxes.hpp"
#include "collisions.hpp"

struct UnresolvedCollision {
    std::shared_ptr<Body> a;
    std::shared_ptr<Body> b;

    double time;
    Vec touch_point;
    Vec normal;
};

struct ResolvedCollision {
    std::shared_ptr<Body> body;
    std::shared_ptr<Body> other;
    double time;
    Vec touch_point;
    Vec impulse;
    Vec closing_velocity;

    void apply_impulse();
};

class World {
public:
    World(double frame_time_=1);
    void addBody(std::shared_ptr<Body> body);
    void removeBody(std::shared_ptr<Body> body);
    void beginFrame();
    UnresolvedCollision nextCollision();
    std::pair<ResolvedCollision, ResolvedCollision> calculateCollision(const UnresolvedCollision& collision, const CollisionParameters& params);
    void finishedCollision(const UnresolvedCollision& collision, bool renotify);
    bool hasNextCollision();
    void endFrame();
    const std::vector<std::shared_ptr<Body>>& bodies() const {
        return _bodies;
    }
private:
    double time_until = 0, current_time = 0, frame_time;
    std::vector<std::shared_ptr<Body>> _bodies;
    std::vector<SigObject> sigobjs;
    std::map<Body*, double> body_times;
    std::set<Body*> changed_bodies, removed_bodies;
    std::map<std::pair<Body*, Body*>, bool> ignore_current_collision;
    std::vector<std::tuple<CollisionTimeResult, Shape*, Body*, Shape*, Body*>> collision_times;
    SATAxes sat_axes;

    void _updateCollisionTimes(bool initial);
    void _updateBodySensorView(Body* body);
};

#endif // COLLIDER_HPP
