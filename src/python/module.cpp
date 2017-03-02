#include <iostream>
#include <boost/python.hpp>

#include "utils.hpp"
#include "module.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;


BOOST_PYTHON_MODULE(physics) {
    cout.sync_with_stdio(true);
    python::def("norm_rad", norm_rad);
    python::def("norm_deg", norm_deg);
    python::def("to_deg", to_deg);
    python::def("to_rad", to_rad);
    wrap_vec();
    wrap_collisions();
    wrap_body();
    wrap_sensors();
    wrap_world();
}
