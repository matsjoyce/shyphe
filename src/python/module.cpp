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
using namespace shyphe;

BOOST_PYTHON_MODULE(shyphe) {
    cout.sync_with_stdio(true);
    python::def("norm_rad", norm_rad);
    python::def("norm_deg", norm_deg);
    python::def("to_deg", to_deg);
    python::def("to_rad", to_rad);
    python::def("angle_diff_deg", angle_diff_deg);
    python::def("angle_diff_rad", angle_diff_rad);

    python::scope().attr("pi") = pi();
    python::scope().attr("hpi") = hpi();
    python::scope().attr("dpi") = dpi();

    python::scope().attr("__version__") = metadata().version;
    python::scope().attr("__hexversion__") = metadata().hexversion;
    python::scope().attr("__author__") = metadata().author;
    python::scope().attr("__author_email__") = metadata().author_email;
    python::scope().attr("__copyright__") = metadata().copyright;
    python::scope().attr("__credits__") = python::str(metadata().credits).split(", ");
    python::scope().attr("__license__") = metadata().license;
    python::scope().attr("__maintainer__") = metadata().maintainer;
    python::scope().attr("__email__") = metadata().email;
    python::scope().attr("__status__") = metadata().status;

    wrap_vec();
    wrap_collisions();
    wrap_body();
    wrap_sensors();
    wrap_world();

#ifdef COVERAGE
    wrap_coverage();
#endif
}
