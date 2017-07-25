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

#include "world.hpp"

#include <algorithm>
#include <numeric>
#include <random>

using namespace std;
using namespace shyphe;

pair<Body*, Body*> make_body_pair(Body* a, Body* b) {
    return (a < b) ? make_pair(a, b) : make_pair(b, a);
}

World::World(double frame_time_/*=1*/) : time_until(frame_time_), frame_time(frame_time_) {
}

void World::beginFrame() {
    sigobjs.clear();
    sigobjs.reserve(_bodies.size());
    for (const auto& body: _bodies) {
        sigobjs.push_back({body->position(), body->signature(), body.get()});
    }
    for (const auto body : _bodies) {
        _updateBodySensorView(body.get());
    }
    _updateCollisionTimes(true);
}

void World::endFrame() {
    for (auto body : _bodies) {
        body->update(time_until - body_times[body.get()]);
        body_times[body.get()] = time_until;
    }
    current_time = time_until;
    time_until = current_time + frame_time;
}

void World::addBody(shared_ptr<Body> body) {
    _bodies.push_back(body);
    body_times[body.get()] = current_time;
    changed_bodies.insert(body.get());
}

void World::removeBody(shared_ptr<Body> body) {
    body_times.erase(body.get());
    removed_bodies.insert(body.get());
    changed_bodies.erase(body.get());
    _bodies.erase(remove(_bodies.begin(), _bodies.end(), body), _bodies.end());
    auto pred = [this, &body](const tuple<CollisionTimeResult, Shape*, Body*, Shape*, Body*>& col)
                {return body.get() == get<2>(col) || body.get() == get<4>(col);};
    collision_times.erase(remove_if(collision_times.begin(), collision_times.end(), pred), collision_times.end());
}

void World::_updateCollisionTimes(bool initial) {
    if (initial) {
        sat_axes.reset(_bodies.size());
        for (auto body : _bodies) {
            sat_axes.addBody(body.get(), frame_time);
        }
    }
    else {
        for (auto body : changed_bodies) {
            sat_axes.removeBody(body);
            sat_axes.addBody(body, time_until - body_times[body]);
        }
        for (auto body : removed_bodies) {
            sat_axes.removeBody(body);
        }
    }
    auto possibleCollisions = sat_axes.possibleCollisions();
    for (const auto& poscol : possibleCollisions) {
        if (!initial && !changed_bodies.count(poscol.first) && !changed_bodies.count(poscol.second)) {
            continue;
        }

        double time_window = frame_time, start_time = current_time;
        BodyState a_state = poscol.first->state(), b_state = poscol.second->state();
        if (!initial) {
            start_time = max(body_times[poscol.first], body_times[poscol.second]);
            poscol.first->update(start_time - body_times[poscol.first]);
            poscol.second->update(start_time - body_times[poscol.second]);
            time_window = time_until - start_time;
        }
        auto p = make_body_pair(poscol.second, poscol.first);
        CollisionTimeResult colresult;
        Shape* a;
        Shape* b;
        tie(colresult, a, b) = poscol.first->collide(poscol.second, time_window, ignore_current_collision[p]);

        if (!initial) {
            poscol.first->reset(a_state);
            poscol.second->reset(b_state);
        }

        if (colresult.time == -1) {
            continue;
        }
        colresult.time += start_time;
        auto collision = make_tuple(colresult, a, poscol.first, b, poscol.second);
        // Put in reverse order to allow pop from back
        auto pos = upper_bound(collision_times.begin(), collision_times.end(), collision,
                               [](const tuple<CollisionTimeResult, Shape*, Body*, Shape*, Body*>& a,
                                  const tuple<CollisionTimeResult, Shape*, Body*, Shape*, Body*>& b)
                               {return get<0>(a).time > get<0>(b).time;});
        collision_times.insert(pos, collision);
    }
    if (!initial) {
        changed_bodies.clear();
        removed_bodies.clear();
    }
}

bool World::hasNextCollision() {
    return collision_times.size();
}

UnresolvedCollision World::nextCollision() {
    if (!collision_times.size()) {
        throw runtime_error("No collisions! Check has_next_collision first!");
    }

    CollisionTimeResult colresult;
    Shape* a;
    Body* a_body;
    Shape* b;
    Body* b_body;

    tie(colresult, a, a_body, b, b_body) = collision_times.back();
    collision_times.pop_back();
    changed_bodies.insert(a_body);
    changed_bodies.insert(b_body);
    a_body->update(colresult.time - body_times[a_body]);
    b_body->update(colresult.time - body_times[b_body]);
    body_times[a_body] = body_times[b_body] = colresult.time;
    return {a_body->shared_from_this(), b_body->shared_from_this(), colresult.time, colresult.touch_point, colresult.normal};
}

std::pair<ResolvedCollision, ResolvedCollision> World::calculateCollision(const UnresolvedCollision& collision, const CollisionParameters& params) {
    auto cr = collisionResult({collision.time, collision.touch_point, collision.normal}, *collision.a, *collision.b, params);
    return {ResolvedCollision{collision.a,
                              collision.b,
                              collision.time,
                              collision.touch_point - collision.a->position(),
                              cr.impulse,
                              cr.closing_velocity
                              },
            ResolvedCollision{collision.b,
                              collision.a,
                              collision.time,
                              collision.touch_point - collision.b->position(),
                              -cr.impulse,
                              -cr.closing_velocity
                              }};
}

void World::finishedCollision(const UnresolvedCollision& collision, bool renotify) {
    ignore_current_collision[make_body_pair(collision.a.get(), collision.b.get())] = !renotify;
    auto pred = [this](const tuple<CollisionTimeResult, Shape*, Body*, Shape*, Body*>& col)
                {return changed_bodies.count(get<2>(col)) || changed_bodies.count(get<4>(col));};
    collision_times.erase(remove_if(collision_times.begin(), collision_times.end(), pred), collision_times.end());
    _updateCollisionTimes(false);
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
        for (const auto& sensor : body->_sensors) {
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
                            sig.body->shared_from_this()});
    }
    shuffle(new_scan.begin(), new_scan.end(), ranlux48());
    set<SensedObject*> new_scan_copy;
    for (auto& so : new_scan) {
        new_scan_copy.insert(&so);
    }
    for (auto& so : old_scan) {
        so.position += so.velocity * frame_time;
    }
    auto cmp = [](const SensedObject& l, const SensedObject& r){return l.position.x < r.position.x;};
    sort(old_scan.begin(), old_scan.end(), cmp);
    for (OldScanMergeCmp cmp2{16}; cmp2.search_radius <= 1024 && new_scan_copy.size() && old_scan.size(); cmp2.search_radius <<= 1) {
        for (auto& so : new_scan_copy) {
            auto start = lower_bound(old_scan.begin(), old_scan.end(), so, cmp2);
            auto end = upper_bound(start, old_scan.end(), so, cmp2);
            for (; start != end; ++start) {
                if (fabs(start->position.y - so->position.y) > cmp2.search_radius) {
                    continue;
                }
                if (so->signature.approx_equals(start->signature, 0.9)) {
                    so->velocity = so->position - start->position - start->velocity * frame_time;
                    new_scan_copy.erase(so);
                    old_scan.erase(start);
                    break;
                }
            }
        }
    }
}

void ResolvedCollision::apply_impulse() {
    body->applyImpulse(impulse, touch_point);
}
