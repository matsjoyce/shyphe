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
        sigobjs.push_back({body->position(), body->signature(), body});
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
                changed_bodies.insert(col.a);
                changed_bodies.insert(col.b);
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
        auto p = poscol.first < poscol.second ? make_pair(poscol.first, poscol.second) : make_pair(poscol.second, poscol.first);
        auto colresult = poscol.first->collide(poscol.second, time_until, !overlapping.count(p));
        if (colresult.time == -1) {
            continue;
        }
        auto pos = upper_bound(collision_times.begin(), collision_times.end(), colresult,
                               [](const CollisionTimeResult& a, const CollisionTimeResult& b){return a.time > b.time;});
        collision_times.insert(pos, colresult);
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

        auto p = poscol.first < poscol.second ? make_pair(poscol.first, poscol.second) : make_pair(poscol.second, poscol.first);
        auto is_overlapping = overlapping.count(p);
        auto colresult = poscol.first->collide(poscol.second, time_until, !is_overlapping);

        if (colresult.time == -1) {
            continue;
        }
        colresult.time += start_time;
        // Put in reverse order to allow pop from back
        auto pos = upper_bound(collision_times.begin(), collision_times.end(), colresult,
                               [](const CollisionTimeResult& a, const CollisionTimeResult& b){return a.time > b.time;});
        collision_times.insert(pos, colresult);

        poscol.first->updatePosition(body_times[poscol.first] - start_time);
        poscol.second->updatePosition(body_times[poscol.second] - start_time);
    }
    _updateCollisionTimesCommon();
}

bool World::hasNextCollision() {
    return collision_times.size();
}

CollisionTimeResult World::nextCollision() {
    if (!collision_times.size()) {
        throw runtime_error("No collisions! Check has_next_collision first!");
    }
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
                      collision.touch_point - collision.a->position(),
                      cr.impulse,
                      cr.closing_velocity
                      },
            Collision{collision.b,
                      collision.a,
                      collision.time,
                      collision.touch_point - collision.b->position(),
                      -cr.impulse,
                      -cr.closing_velocity
                      }};
}

void World::finishedCollision(const CollisionTimeResult& collision, bool renotify) {
    auto pred = [this](const CollisionTimeResult& col){return changed_bodies.count(col.a) || changed_bodies.count(col.b);};
    collision_times.erase(remove_if(collision_times.begin(), collision_times.end(), pred), collision_times.end());
    if (!renotify) {
        if (collision.a < collision.b) {
            overlapping.insert({collision.a, collision.b});
        }
        else {
            overlapping.insert({collision.b, collision.a});
        }
    }
    _updateCollisionTimesChanged();
}

struct OldScanMergeCmp {
    int search_radius;
    bool operator()(const SensedObject& c, const SensedObject* v) {
        return c.position.x < v->position.x - search_radius;
    }
    bool operator()(const SensedObject* v, const SensedObject& c) {
        return c.position.x < v->position.x - search_radius;
    }
};

void World::_updateBodySensorView(Body* body) {
    vector<SensedObject> old_scan;
    swap(old_scan, body->_sensor_view);
    vector<SensedObject>& new_scan = body->_sensor_view;
    bool has_indentifier;
    for (const auto& sig : sigobjs) {
        if (sig.body == body) {
            continue;
        }
        Signature signature;
        has_indentifier = false;
        double dist = (body->position() - sig.body->position()).abs() + 0.00001;
        for (const auto& sensor : body->sensors) {
            if (dist > sensor->maxRange()) {
                continue;
            }
            auto sensed_signature = sensor->intensity(sig, dist);
            if (sensed_signature) {
                signature &= sensed_signature;
                has_indentifier = has_indentifier || sensor->givesIdentification();
            }
        }
        if (!signature) {
            continue;
        }
        auto side = SensedObject::unknown;
        if (has_indentifier) {
            if (!sig.body->side()) {
                side = SensedObject::neutral;
            }
            else if (body->side() == sig.body->side()) {
                side = SensedObject::friendly;
            }
            else {
                side = SensedObject::enemy;
            }
        }
        new_scan.push_back({sig.body->position() - body->position(),
                            {0, 0},
                            signature,
                            side,
                            sig.body});
    }
    shuffle(new_scan.begin(), new_scan.end(), ranlux48());
    set<SensedObject*> new_scan_copy;
    for (auto& so : new_scan) {
        new_scan_copy.insert(&so);
    }
    for (auto& so : old_scan) {
        so.position += so.velocity * time_until;
    }
    auto cmp = [](const SensedObject& l, const SensedObject& r){return l.position.x < r.position.x;};
    sort(old_scan.begin(), old_scan.end(), cmp);
    auto cmp2 = OldScanMergeCmp{16};
    for (; cmp2.search_radius <= 1024 && new_scan_copy.size() && old_scan.size(); cmp2.search_radius <<= 1) {
        for (auto& so : new_scan_copy) {
            auto start = lower_bound(old_scan.begin(), old_scan.end(), so, cmp2);
            auto end = upper_bound(start, old_scan.end(), so, cmp2);
            for (; start != end; ++start) {
                if (fabs(start->position.y - so->position.y) > cmp2.search_radius) {
                    continue;
                }
                if (so->signature.approx_equals(start->signature, 0.9)) {
                    so->velocity = so->position - start->position - start->velocity * time_until;
                    new_scan_copy.erase(so);
                    old_scan.erase(start);
                    break;
                }
            }
        }
    }
}
