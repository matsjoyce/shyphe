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
#include "pair_support.hpp"
#include "container_support.hpp"
#include "world.hpp"

using namespace std;
using namespace shyphe;

void wrap_world() {
    python::class_<World>("World", python::init<double>())
        .def("add_body", &World::addBody)
        .def("remove_body", &World::removeBody)
        .def("begin_frame", &World::beginFrame)
        .def("end_frame", &World::endFrame)
        .def("next_collision", &World::nextCollision)
        .def("calculate_collision", &World::calculateCollision)
        .def("finished_collision", &World::finishedCollision)
        .def("has_next_collision", &World::hasNextCollision)
        .add_property("bodies", python::make_function(&World::bodies, python::return_internal_reference<>()));
    python::class_<UnresolvedCollision>("UnresolvedCollision", python::no_init)//, python::init<Body*, Body*, Shape*, Shape*, double, Vec, Vec>())
        .def_readonly("a", &UnresolvedCollision::a)
        .def_readonly("b", &UnresolvedCollision::b)
        .def_readonly("time", &UnresolvedCollision::time)
        .def_readonly("touch_point", &UnresolvedCollision::touch_point)
        .def_readonly("normal", &UnresolvedCollision::normal);
    python::class_<ResolvedCollision>("ResolvedCollision")
        .def_readonly("body", &ResolvedCollision::body)
        .def_readonly("other", &ResolvedCollision::other)
        .def_readonly("time", &ResolvedCollision::time)
        .def_readonly("touch_point", &ResolvedCollision::touch_point)
        .add_property("impulse",
             python::make_getter(&ResolvedCollision::impulse, python::return_value_policy<python::return_by_value>()),
             python::make_setter(&ResolvedCollision::impulse))
        .def_readonly("closing_velocity", &ResolvedCollision::closing_velocity)
        .def("apply_impulse", &ResolvedCollision::apply_impulse);
    PairConverter<ResolvedCollision, ResolvedCollision>();
    ContainerConverter<vector<shared_ptr<Body>>, true>("BodyVector");
}
