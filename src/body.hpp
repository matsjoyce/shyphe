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

#ifndef SHYPHE_BODY_HPP
#define SHYPHE_BODY_HPP

#include <vector>
#include <tuple>
#include <memory>

#include "vec.hpp"
#include "aabb.hpp"
#include "collisions.hpp"
#include "shape.hpp"
#include "sensor.hpp"

namespace shyphe {
    struct BodyState {
        BodyState(Vec position_, Vec velocity_,
                  Vec local_force_, Vec global_force_,
                  double local_torque_, double global_torque_,
                  double angle_, double angular_velocity_) : position(position_), velocity(velocity_),
                                                             local_force(local_force_), global_force(global_force_),
                                                             local_torque(local_torque_), global_torque(global_torque_),
                                                             angle(angle_), angular_velocity(angular_velocity_) {

        }

        Vec position, velocity;
        Vec local_force, global_force;
        double local_torque, global_torque;
        double angle, angular_velocity;

        friend class Body;
    };

    class Body : public std::enable_shared_from_this<Body> {
    public:
        Body(const Vec& position_={}, const Vec& velocity_={},
             double angle_=0, double angular_velocity_=0, int side_=0);
        virtual ~Body() = default;

        AABB aabb(double time) const;
        double mass() const;
        double momentOfInertia() const;

        inline const Vec& position() const {
            return _position;
        }

        inline const Vec& velocity() const {
            return _velocity;
        }

        inline double angle() const {
            return _angle;
        }

        inline double angularVelocity() const {
            return _angular_velocity;
        }

        inline Vec localForce() const {
            return _local_force;
        }

        inline Vec globalForce() const {
            return _global_force;
        }

        inline double localTorque() const {
            return _local_torque;
        }

        inline double globalTorque() const {
            return _global_torque;
        }

        inline int side() const {
            return _side;
        }

        inline const std::vector<SensedObject>& sensorView() const {
            return _sensor_view;
        }

        inline const std::vector<std::shared_ptr<Shape>>& shapes() const {
            return _shapes;
        }

        inline const std::vector<std::shared_ptr<Sensor>>& sensors() const {
            return _sensors;
        }

        void changeSide(int new_side);
        void teleport(const Vec& to);

        void applyImpulse(Vec impulse, Vec position);
        void applyLocalForce(Vec force, Vec position);
        void applyGlobalForce(Vec force, Vec position);
        void clearLocalForces();
        void clearGlobalForces();

        Signature signature();
        void update(double time);
        void addShape(std::shared_ptr<Shape> shape);
        void removeShape(std::shared_ptr<Shape> shape);
        void addSensor(std::shared_ptr<Sensor> shape);
        void removeSensor(std::shared_ptr<Sensor> shape);
        std::tuple<CollisionTimeResult, Shape*, Shape*> collide(Body* other, double end_time, bool ignore_initial) const;
        double distanceBetween(Body* other) const;
        double maxSensorRange() const;

        BodyState state() const;
        void reset(BodyState state);
    private:
        Vec _position, _velocity;
        Vec _local_force = {}, _global_force = {};
        double _local_torque = 0, _global_torque = 0;
        double _angle, _angular_velocity;
        int _side;
        std::vector<SensedObject> _sensor_view;

        std::vector<std::shared_ptr<Shape>> _shapes;
        std::vector<std::shared_ptr<Sensor>> _sensors;

        friend class World;
    };
}

#endif // SHYPHE_BODY_HPP
