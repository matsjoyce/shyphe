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

#include "sataxes.hpp"

#include <algorithm>

using namespace std;

void SATAxes::reset(int reserve_hint)
{
    x_axis.clear();
    y_axis.clear();

    if (reserve_hint) {
        x_axis.reserve(reserve_hint);
        y_axis.reserve(reserve_hint);
    }
}

void SATAxes::addBody(Body* body, double time) {
    auto aabb = body->position() + body->aabb(time);

    auto cmp = [](const SATShadow& a, const SATShadow& b){return a.position < b.position;};

    auto tmp = SATShadow{aabb.min_x, true, body};
    auto minpos = upper_bound(x_axis.begin(), x_axis.end(), tmp, cmp);
    x_axis.insert(minpos, move(tmp));

    tmp = SATShadow{aabb.max_x, false, body};
    auto maxpos = upper_bound(x_axis.begin(), x_axis.end(), tmp, cmp);
    x_axis.insert(maxpos, move(tmp));

    tmp = SATShadow{aabb.min_y, true, body};
    minpos = upper_bound(y_axis.begin(), y_axis.end(), tmp, cmp);
    y_axis.insert(minpos, move(tmp));

    tmp = SATShadow{aabb.max_y, false, body};
    maxpos = upper_bound(y_axis.begin(), y_axis.end(), tmp, cmp);
    y_axis.insert(maxpos, move(tmp));
}

void SATAxes::removeBody(Body* body) {
    auto pred = [body](const SATShadow& sh){return sh.body == body;};

    x_axis.erase(remove_if(x_axis.begin(), x_axis.end(), pred), x_axis.end());
    y_axis.erase(remove_if(y_axis.begin(), y_axis.end(), pred), y_axis.end());
}

set<pair<Body*, Body*>> SATAxes::_collisionsOnAxis(const vector<SATShadow>& axis) {
    auto stack = set<Body*>{};
    auto result = set<pair<Body*, Body*>>{};
    for (const auto& shadow : axis) {
        if (shadow.start) {
            for (const auto& other : stack) {
                if (other < shadow.body) {
                    result.insert({other, shadow.body});
                }
                else {
                    result.insert({shadow.body, other});
                }
            }
            stack.insert(shadow.body);
        }
        else {
            stack.erase(shadow.body);
        }
    }
    return result;
}

set<pair<Body*, Body*>> SATAxes::possibleCollisions() {
    auto xs = _collisionsOnAxis(x_axis);
    auto ys = _collisionsOnAxis(y_axis);
    auto result = set<pair<Body*, Body*>>{};
    set_intersection(xs.begin(), xs.end(), ys.begin(), ys.end(), inserter(result, result.begin()));
    return result;
}
