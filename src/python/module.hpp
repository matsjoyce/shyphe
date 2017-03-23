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
