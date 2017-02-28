/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
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

#include <algorithm>
#include <random>
#include <numeric>
#include "sensormap.hpp"
#include "body.hpp"

using namespace std;

void SensorMap::addBody(Body* body) {
    bodies.push_back(body);
}

void SensorMap::removeBody(Body* body) {
    bodies.erase(remove(bodies.begin(), bodies.end(), body), bodies.end());
}

void SensorMap::reset() {
    sigobjs.clear();
    sigobjs.reserve(bodies.size());
    for (const auto& body: bodies) {
        sigobjs.push_back({body->position, body->signature(), body});
    }
}

void SensorMap::scan(Body* body) {
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
