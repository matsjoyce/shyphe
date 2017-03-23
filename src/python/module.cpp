#include <iostream>

#include "utils.hpp"
#include "module.hpp"

using namespace std;

BOOST_PYTHON_MODULE(physics) {
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
