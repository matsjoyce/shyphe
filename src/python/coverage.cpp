#include <boost/python.hpp>

#include "module.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;

// https://www.osadl.org/Dumping-gcov-data-at-runtime-simple-ex.online-coverage-analysis.0.html

extern "C" {
    void __gcov_flush(void);
}

void flush_coverage() {
    __gcov_flush();
}

void wrap_coverage() {
    python::def("flush_coverage", &flush_coverage);
}