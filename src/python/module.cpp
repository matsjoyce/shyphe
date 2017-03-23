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

#include <iostream>

#include "utils.hpp"
#include "module.hpp"

using namespace std;

BOOST_PYTHON_MODULE(shyphe) {
    cout.sync_with_stdio(true);
    python::def("norm_rad", norm_rad);
    python::def("norm_deg", norm_deg);
    python::def("to_deg", to_deg);
    python::def("to_rad", to_rad);

    python::scope().attr("pi") = pi();
    python::scope().attr("hpi") = hpi();
    python::scope().attr("dpi") = dpi();

    wrap_vec();
    wrap_collisions();
    wrap_body();
    wrap_sensors();
    wrap_world();

#ifdef COVERAGE
    wrap_coverage();
#endif
}
