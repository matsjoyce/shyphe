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

#include "world.hpp"

#include <numeric>
#include <random>

using namespace std;

void World::beginFrame(double time) {
    time_until = time;
    current_time = 0;
    body_times.clear();
    sigobjs.clear();
    sigobjs.reserve(bodies.size());
    for (const auto& body: bodies) {
        sigobjs.push_back({body->position, body->signature(), body});
    }
    for (const auto body : bodies) {
        body_times[body] = 0;
        _updateBodySensorView(body);
    }
    _updateCollisionTimes();
}

void World::endFrame() {
    for (auto body : bodies) {
        body->updatePosition(time_until - body_times[body]);
        body->updateVelocity(time_until);
    }
}

void World::addBody(Body* body) {
    bodies.push_back(body);
    body_times[body] = 0;
    changed_bodies.insert(body);
}

void World::removeBody(Body* body) {
    bodies.erase(remove(bodies.begin(), bodies.end(), body), bodies.end());
    body_times.erase(body);
    removed_bodies.insert(body);
    changed_bodies.erase(body);
}

void World::_updateCollisionTimesCommon() {
    changed_bodies.clear();
    removed_bodies.clear();
    while (collision_times.size()) {
        const auto& col = collision_times.back();
        auto p = col.a < col.b ? make_pair(col.a, col.b) : make_pair(col.b, col.a);
        if (overlapping.count(p)) {
            if (!col.entering) {
                overlapping.erase(p);
            }
            collision_times.pop_back();
        }
        else if (!col.entering) {
            collision_times.pop_back();
        }
        else {
            break;
        }
    }
}

void World::_updateCollisionTimes() {
    sat_axes.reset(bodies.size());
    for (auto body : bodies) {
        sat_axes.addBody(body, time_until);
    }
    auto possibleCollisions = sat_axes.possibleCollisions();
    for (const auto& poscol : possibleCollisions) {
        auto colresult = poscol.first->collide(poscol.second, time_until);
        if (colresult.time == -1) {
            continue;
        }
        auto pos = upper_bound(collision_times.begin(), collision_times.end(), colresult,
                               [](const CollisionTimeResult& a, const CollisionTimeResult& b){return a.time > b.time;});
        collision_times.insert(pos, move(colresult));
    }
    _updateCollisionTimesCommon();
}

void World::_updateCollisionTimesChanged() {
    for (auto body : removed_bodies) {
        sat_axes.removeBody(body);
    }
    for (auto body : changed_bodies) {
        sat_axes.removeBody(body);
        sat_axes.addBody(body, time_until - body_times[body]);
    }
    auto possibleCollisions = sat_axes.possibleCollisions();
    for (const auto& poscol : possibleCollisions) {
        if (!changed_bodies.count(poscol.first) && !changed_bodies.count(poscol.second)) {
            continue;
        }
        auto start_time = max(body_times[poscol.first], body_times[poscol.second]);
        poscol.first->updatePosition(start_time - body_times[poscol.first]);
        poscol.second->updatePosition(start_time - body_times[poscol.second]);

        auto colresult = poscol.first->collide(poscol.second, time_until);

        poscol.first->updatePosition(body_times[poscol.first] - start_time);
        poscol.second->updatePosition(body_times[poscol.second] - start_time);

        if (colresult.time == -1) {
            continue;
        }
        colresult.time += start_time;
        // Put in reverse order to allow pop from back
        auto pos = upper_bound(collision_times.begin(), collision_times.end(), colresult,
                               [](const CollisionTimeResult& a, const CollisionTimeResult& b){return a.time > b.time;});
        collision_times.insert(pos, move(colresult));
    }
    _updateCollisionTimesCommon();
}

bool World::hasNextCollision() {
    return collision_times.size();
}

CollisionTimeResult World::nextCollision() {
    auto collision = collision_times.back();
    collision_times.pop_back();
    changed_bodies.insert(collision.a);
    changed_bodies.insert(collision.b);
    collision.a->updatePosition(collision.time - body_times[collision.a]);
    collision.b->updatePosition(collision.time - body_times[collision.b]);
    body_times[collision.a] = body_times[collision.b] = collision.time;
    return collision;
}

std::pair<Collision, Collision> World::calculateCollision(const CollisionTimeResult& collision, const CollisionParameters& params) {
    auto cr = collisionResult(collision, params);
    return {Collision{collision.a,
                      collision.b,
                      collision.time,
                      collision.touch_point - collision.a->position,
                      cr.impulse,
                      cr.closing_velocity
                      },
            Collision{collision.b,
                      collision.a,
                      collision.time,
                      collision.touch_point - collision.b->position,
                      -cr.impulse,
                      -cr.closing_velocity
                      }};
}

void World::finishedCollision(const pair<Collision, Collision>& collisions, bool renotify) {
    auto pred = [this](const CollisionTimeResult& col){return changed_bodies.count(col.a) || changed_bodies.count(col.b);};
    collision_times.erase(remove_if(collision_times.begin(), collision_times.end(), pred), collision_times.end());
    if (!renotify) {
        if (collisions.first.body < collisions.second.body) {
            overlapping.insert({collisions.first.body, collisions.second.body});
        }
        else {
            overlapping.insert({collisions.second.body, collisions.first.body});
        }
    }
    _updateCollisionTimesChanged();
}

void World::_updateBodySensorView(Body* body) {
    vector<SensedObject> old_scan;
    swap(old_scan, body->sensor_view);
    vector<SensedObject>& new_scan = body->sensor_view;
    vector<double> intensities;
    bool has_indentifier;
    for (const auto& sig : sigobjs) {
        if (sig.body == body) {
            continue;
        }
        intensities.clear();
        intensities.reserve(body->sensors.size());
        has_indentifier = false;
        double dist = (body->position - sig.body->position).abs() + 0.00001;
        for (const auto& sensor : body->sensors) {
            if (dist > sensor->maxRange()) {
                continue;
            }
            auto intensity = sensor->intensity(sig, dist);
            if (intensity) {
                intensities.push_back(intensity);
                has_indentifier = has_indentifier || sensor->givesIdentification();
            }
        }
        if (!intensities.size()) {
            continue;
        }
        auto side = SensedObject::unknown;
        if (has_indentifier) {
            if (!sig.body->side) {
                side = SensedObject::neutral;
            }
            else if (body->side == sig.body->side) {
                side = SensedObject::friendly;
            }
            else {
                side = SensedObject::enemy;
            }
        }
        new_scan.push_back({sig.body->position - body->position,
                            {0, 0},
                            accumulate(intensities.begin(), intensities.end(), 0.0) / intensities.size(),
                            side,
                            sig.body});
    }
    shuffle(new_scan.begin(), new_scan.end(), ranlux48());
}
