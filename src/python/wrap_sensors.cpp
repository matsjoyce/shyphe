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
#include "body.hpp"
#include "sensor.hpp"

using namespace std;

void wrap_sensors() {
    SharedConverter<Sensor>();
    python::class_<Sensor, boost::noncopyable, py_ptr<Sensor>>("Sensor", python::no_init)
        .add_property("max_range", &Sensor::maxRange)
        .def_readwrite("perf", &Sensor::perf)
        .def("clone", &Sensor::clone, python::return_value_policy<python::manage_new_object>());
    python::class_<ActiveRadar, boost::noncopyable, python::bases<Sensor>, py_ptr<ActiveRadar>>("ActiveRadar",
        python::init<double, double>((python::arg("power")=0,
                                      python::arg("sensitivity")=0)))
        .def_readwrite("power", &ActiveRadar::power)
        .def_readwrite("sensitivity", &ActiveRadar::sensitivity);
    python::class_<PassiveRadar, boost::noncopyable, python::bases<Sensor>, py_ptr<PassiveRadar>>("PassiveRadar",
        python::init<double>(python::arg("sensitivity")=0))
        .def_readwrite("sensitivity", &PassiveRadar::sensitivity);
    python::class_<PassiveThermal, boost::noncopyable, python::bases<Sensor>, py_ptr<PassiveThermal>>("PassiveThermal",
        python::init<double>(python::arg("sensitivity")=0))
        .def_readwrite("sensitivity", &PassiveThermal::sensitivity);
    python::enum_<SensedObject::Side>("Side")
        .value("friendly", SensedObject::Side::friendly)
        .value("enemy", SensedObject::Side::enemy)
        .value("neutral", SensedObject::Side::neutral)
        .value("unknown", SensedObject::Side::unknown);
    python::class_<SensedObject, boost::noncopyable>("SensedObject", python::init<Vec, Vec, Signature, SensedObject::Side, shared_ptr<Body>>())
        .def(op::self == op::self)
        .def_readonly("position", &SensedObject::position)
        .def_readonly("velocity", &SensedObject::velocity)
        .def_readonly("signature", &SensedObject::signature)
        .def_readonly("side", &SensedObject::side)
        .def_readonly("body", &SensedObject::body);
}
