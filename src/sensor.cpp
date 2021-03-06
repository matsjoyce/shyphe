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

#include "sensor.hpp"
#include <tuple>

using namespace std;
using namespace shyphe;

bool SensedObject::operator==(const SensedObject& other) {
    return tie(position, velocity, signature, side, body) == tie(other.position, other.velocity, other.signature, other.side, other.body);
}


ActiveRadar::ActiveRadar(double pwr/*=0*/, double sens/*=0*/) : power(pwr), sensitivity(sens) {
}

Signature ActiveRadar::intensity(const SigObject& signature, double dist) const {
    if (signature.sig.radar_cross_section * power / dist / 2 * perf < sensitivity) {
        return {};
    }
    return {0, 0, signature.sig.radar_cross_section};
}

bool ActiveRadar::givesIdentification() const {
    return true;
}

double ActiveRadar::maxRange() const {
    return power * 25 / sensitivity * perf;
}

Sensor* ActiveRadar::clone() const {
    return new ActiveRadar{power, sensitivity};
}

PassiveRadar::PassiveRadar(double sens/*=0*/) : sensitivity(sens) {
}

Signature PassiveRadar::intensity(const SigObject& signature, double dist) const {
    if (signature.sig.radar_emissions / dist * perf < sensitivity) {
        return {};
    }
    return {signature.sig.radar_emissions, 0, 0};
}

bool PassiveRadar::givesIdentification() const {
    return false;
}

double PassiveRadar::maxRange() const {
    return 50 / sensitivity * perf;
}

Sensor* PassiveRadar::clone() const {
    return new PassiveRadar{sensitivity};
}

PassiveThermal::PassiveThermal(double sens/*=0*/) : sensitivity(sens) {
}

Signature PassiveThermal::intensity(const SigObject& signature, double dist) const {
    if (signature.sig.thermal_emissions / dist * perf < sensitivity) {
        return {};
    }
    return {0, signature.sig.thermal_emissions, 0};
}

bool PassiveThermal::givesIdentification() const {
    return false;
}

double PassiveThermal::maxRange() const {
    return 2500 / sensitivity * perf;
}

Sensor* PassiveThermal::clone() const {
    return new PassiveThermal{sensitivity};
}
