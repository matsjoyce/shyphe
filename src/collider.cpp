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

#include "collider.hpp"

using namespace std;

const double ANTI_REPEAT_TIME = 0.0001;

void Collider::reset(double time) {
    time_until = time;
    current_time = 0;
    body_times.clear();
    for (const auto body : bodies) {
        body_times[body] = 0;
    }
    _updateCollisionTimes();
}

void Collider::addBody(Body* body) {
    bodies.push_back(body);
    body_times[body] = 0;
    changed_bodies.insert(body);
}

void Collider::removeBody(Body* body) {
    bodies.erase(remove(bodies.begin(), bodies.end(), body), bodies.end());
    body_times.erase(body);
    removed_bodies.insert(body);
    changed_bodies.erase(body);
}

void Collider::_updateCollisionTimesCommon() {
    changed_bodies.clear();
    removed_bodies.clear();
    if (!hasNextCollision()) {
        for (auto body : bodies) {
            body->updatePosition(time_until - body_times[body]);
        }
    }
    else {
        while (collision_times.size()) {
            const auto& col = collision_times.back();
            auto p = col.a < col.b ? make_pair(col.a, col.b) : make_pair(col.b, col.a);
            if (overlapping.count(p)) {
                overlapping.erase(p);
                collision_times.pop_back();
            }
            else {
                break;
            }
        }
    }
}

void Collider::_updateCollisionTimes() {
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

void Collider::_updateCollisionTimesChanged() {
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

bool Collider::hasNextCollision() {
    return collision_times.size();
}

pair<Collision, Collision> Collider::nextCollision() {
    auto collision = collision_times.back();
    collision_times.pop_back();
    changed_bodies.insert(collision.a);
    changed_bodies.insert(collision.b);
    collision.a->updatePosition(collision.time - body_times[collision.a]);
    collision.b->updatePosition(collision.time - body_times[collision.b]);
    body_times[collision.a] = body_times[collision.b] = collision.time;
    auto cr = collisionResult(collision, 1, 1000, 0.1);
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

void Collider::finishedCollision(const pair<Collision, Collision>& collisions) {
    auto pred = [this](const CollisionTimeResult& col){return changed_bodies.count(col.a) || changed_bodies.count(col.b);};
    collision_times.erase(remove_if(collision_times.begin(), collision_times.end(), pred), collision_times.end());
    for (auto& body : changed_bodies) {
        body->updatePosition(ANTI_REPEAT_TIME);
        body_times[body] += ANTI_REPEAT_TIME;
    }
    if (collisions.first.body->immediate_collide(collisions.second.body)) {
        if (collisions.first.body < collisions.second.body) {
            overlapping.insert({collisions.first.body, collisions.second.body});
        }
        else {
            overlapping.insert({collisions.second.body, collisions.first.body});
        }
    }
    _updateCollisionTimesChanged();
}
