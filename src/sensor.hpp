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

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "vec.hpp"
#include "shape.hpp"

class Body;

struct SigObject {
    Vec position;
    Signature sig;
    Body* body;
};

struct SensedObject {
    enum Side {
        friendly,
        enemy,
        neutral,
        unknown
    };

    constexpr SensedObject(const Vec& pos, const Vec& vel, double intensity_, Side side_, Body* body_) : position(pos),
                                                                                                         velocity(vel),
                                                                                                         intensity(intensity_),
                                                                                                         side(side_),
                                                                                                         body(body_) {
    }
    bool operator==(const SensedObject& other);

    Vec position, velocity;
    double intensity;
    Side side;
    Body* body;
};

class Sensor {
public:
    virtual double intensity(const SigObject& signature, double dist) const = 0;
    virtual bool givesIdentification() const = 0;
    virtual double maxRange() const = 0;
    virtual Sensor* clone() const = 0;

    Body* body = nullptr;
};

class ActiveRadar : public Sensor {
public:
    ActiveRadar(double pwr=0, double sens=0);
    virtual double intensity(const SigObject& signature, double dist) const override;
    virtual bool givesIdentification() const override;
    virtual double maxRange() const override;
    virtual Sensor* clone() const override;

    double power, sensitivity;
};

class PassiveRadar : public Sensor {
public:
    PassiveRadar(double sens=0);
    virtual double intensity(const SigObject& signature, double dist) const override;
    virtual bool givesIdentification() const override;
    virtual double maxRange() const override;
    virtual Sensor* clone() const override;

    double sensitivity;
};

class PassiveThermal : public Sensor {
public:
    PassiveThermal(double sens=0);
    virtual double intensity(const SigObject& signature, double dist) const override;
    virtual bool givesIdentification() const override;
    virtual double maxRange() const override;
    virtual Sensor* clone() const override;

    double sensitivity;
};

#endif // SENSOR_HPP
