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

#ifndef PYTHON_MODULE_HPP
#define PYTHON_MODULE_HPP

#include <boost/python.hpp>

namespace python = boost::python;
namespace op = boost::python::self_ns;

void wrap_vec();
void wrap_collisions();
void wrap_body();
void wrap_sensors();
void wrap_world();

#ifdef COVERAGE
void wrap_coverage();
#endif

#endif // PYTHON_MODULE_HPP
