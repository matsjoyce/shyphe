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

#include "module.hpp"
#include "shared_ptr_support.hpp"
#include "container_support.hpp"
#include "tuple_support.hpp"
#include "body.hpp"
#include "shape.hpp"
#include "sensor.hpp"
#include "circle.hpp"
#include "massshape.hpp"
#include "polygon.hpp"

using namespace std;

python::tuple sig_as_tuple(const Signature& s) {
    return python::make_tuple(s.radar_emissions, s.thermal_emissions, s.radar_cross_section);
}

python::tuple aabb_as_tuple(const AABB& aabb) {
    return python::make_tuple(aabb.min_x, aabb.max_x, aabb.min_y, aabb.max_y);
}

string aabb_repr(const AABB& aabb) {
    stringstream ss;
    ss << "AABB(" << aabb.min_x << ", " << aabb.max_x << ", " << aabb.min_y << ", " << aabb.max_y << ")";
    return ss.str();
}

tuple<CollisionTimeResult, shared_ptr<Shape>, shared_ptr<Shape>> body_collide(Body& a, Body* b, double et, bool i) {
    CollisionTimeResult ctr;
    Shape* s1;
    Shape* s2;
    tie(ctr, s1, s2) = a.collide(b, et, i);
    return {ctr, s1->shared_from_this(), s2->shared_from_this()};
}

void wrap_body() {
    // Note: All the Vec properties have to return copies to preserve immutability
    SharedConverter<Body>();
    python::class_<Body, boost::noncopyable, py_ptr<Body>>("Body",
        python::init<const Vec&, const Vec&, double, double, int>((python::arg("position")=Vec{},
                                                                   python::arg("velocity")=Vec{},
                                                                   python::arg("angle")=0,
                                                                   python::arg("angular_velocity")=0,
                                                                   python::arg("side")=0)))
        .add_property("position", make_function(&Body::position, python::return_value_policy<python::return_by_value>()))
        .add_property("velocity", make_function(&Body::velocity, python::return_value_policy<python::return_by_value>()))
        .add_property("angle", &Body::angle)
        .add_property("angular_velocity", &Body::angularVelocity)
        .add_property("side", &Body::side)
        .add_property("sensor_view", make_function(&Body::sensorView, python::return_internal_reference<>()))
        .add_property("mass", &Body::mass)
        .add_property("moment_of_inertia", &Body::momentOfInertia)
        .add_property("max_sensor_range", &Body::maxSensorRange)
        .add_property("shapes", python::make_function(&Body::shapes, python::return_internal_reference<>()))
        .add_property("sensors", python::make_function(&Body::sensors, python::return_internal_reference<>()))
        .def("aabb", &Body::aabb)
        .def("update_position", &Body::updatePosition)
        .def("update_velocity", &Body::updateVelocity)
        .def("teleport", &Body::teleport)
        .def("change_side", &Body::changeSide)
        .def("apply_impulse", &Body::applyImpulse)
        .def("apply_local_force", &Body::applyLocalForce)
        .def("clear_local_forces", &Body::clearLocalForces)
        .def("apply_global_force", &Body::applyGlobalForce)
        .def("clear_global_forces", &Body::clearGlobalForces)
        .def("HACK_set_angular_velocity", &Body::HACK_setAngularVelocity)
        .def("collide", body_collide)
        .def("distance_between", &Body::distanceBetween)
        .def("add_shape", &Body::addShape)
        .def("remove_shape", &Body::removeShape)
        .def("add_sensor", &Body::addSensor)
        .def("remove_sensor", &Body::removeSensor);
    python::class_<Signature>("Signature",
        python::init<double, double, double>((python::arg("radar_emissions")=0,
                                              python::arg("thermal_emissions")=0,
                                              python::arg("radar_cross_section")=0)))
        .def_readwrite("radar_emissions", &Signature::radar_emissions)
        .def_readwrite("thermal_emissions", &Signature::thermal_emissions)
        .def_readwrite("radar_cross_section", &Signature::radar_cross_section)
        .def("as_tuple", &sig_as_tuple);
    python::class_<AABB>("AABB", python::init<double, double, double, double>())
        .def(python::init<Vec, double, double>())
        .def(python::init<Vec, Vec>())
        .def_readonly("min_x", &AABB::min_x)
        .def_readonly("min_y", &AABB::min_y)
        .def_readonly("max_x", &AABB::max_x)
        .def_readonly("max_y", &AABB::max_y)
        .add_property("center", &AABB::center)
        .add_property("bottomleft", &AABB::bottomleft)
        .add_property("bottomright", &AABB::bottomright)
        .add_property("topleft", &AABB::topleft)
        .add_property("topright", &AABB::topright)
        .def(op::self + python::other<Vec>())
        .def(python::other<Vec>() + op::self)
        .def(op::self & python::other<AABB>())
        .def(op::str(op::self))
        .def("as_tuple", aabb_as_tuple)
        .def("__repr__", aabb_repr);

    SharedConverter<Shape>();
    python::class_<Shape, boost::noncopyable, py_ptr<Shape>>("Shape", python::no_init)
        .def_readwrite("mass", &Shape::mass)
        .add_property("position",
             python::make_getter(&Shape::position, python::return_value_policy<python::return_by_value>()),
             python::make_setter(&Shape::position))
        .add_property("moment_of_inertia", &Shape::momentOfInertia)
        .def_readwrite("signature", &Shape::signature)
        .def("aabb", &Shape::aabb)
        .def("bounding_radius", &Shape::boundingRadius)
        .def("can_collide", &Shape::canCollide)
        .def("clone", &Shape::clone, python::return_value_policy<python::manage_new_object>());
    python::class_<Circle, boost::noncopyable, python::bases<Shape>, py_ptr<Circle>>("Circle",
        python::init<double, double, const Vec&, double, double, double>((python::arg("radius")=0,
                                                                          python::arg("mass")=0,
                                                                          python::arg("position")=Vec{},
                                                                          python::arg("radar_cross_section")=0,
                                                                          python::arg("radar_emissions")=0,
                                                                          python::arg("thermal_emissions")=0)))
        .def_readwrite("radius", &Circle::radius);
    python::class_<MassShape, boost::noncopyable, python::bases<Shape>, py_ptr<MassShape>>("MassShape",
        python::init<double, const Vec&, double, double, double>((python::arg("mass")=0,
                                                                  python::arg("position")=Vec{},
                                                                  python::arg("radar_cross_section")=0,
                                                                  python::arg("radar_emissions")=0,
                                                                  python::arg("thermal_emissions")=0)));
    python::class_<Polygon, boost::noncopyable, python::bases<Shape>, py_ptr<Polygon>>("Polygon",
        python::init<vector<Vec>, double, const Vec&, double, double, double>((python::arg("points")=python::list(),
                                                                               python::arg("mass")=0,
                                                                               python::arg("position")=Vec{},
                                                                               python::arg("radar_cross_section")=0,
                                                                               python::arg("radar_emissions")=0,
                                                                               python::arg("thermal_emissions")=0)))
        .def_readonly("points", &Polygon::points);
    ContainerConverter<vector<SensedObject>>("SensedObjectVector");
    ContainerConverter<vector<Vec>>("VecVector");
    ContainerConverter<vector<shared_ptr<Shape>>, true>("ShapeVector");
    ContainerConverter<vector<shared_ptr<Sensor>>, true>("SensorVector");
    TupleConverter<CollisionTimeResult, shared_ptr<Shape>, shared_ptr<Shape>>();
}
