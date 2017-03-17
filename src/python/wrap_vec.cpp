#include <boost/python.hpp>
#include <string>
#include <sstream>

#include "module.hpp"
#include "vec.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;

struct Vec_from_tuple {
    Vec_from_tuple() {
        python::converter::registry::push_back(&convertible, &construct, python::type_id<Vec>());
    }

    static void* convertible(PyObject* obj_ptr) {
        if (!PyTuple_Check(obj_ptr)) {
            return 0;
        }
        if (PyTuple_Size(obj_ptr) != 2) {
            PyErr_SetString(PyExc_ValueError, "Vec from tuple: tuple must have length 2");
            return 0;
        }
        auto xptr = PyTuple_GetItem(obj_ptr, 0);
        if (!PyNumber_Check(xptr)) {
            PyErr_SetString(PyExc_ValueError, "Vec from tuple: index 0 is not a number");
            return 0;
        }
        auto yptr = PyTuple_GetItem(obj_ptr, 1);
        if (!PyNumber_Check(yptr)) {
            PyErr_SetString(PyExc_ValueError, "Vec from tuple: index 1 is not a number");
            return 0;
        }
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, python::converter::rvalue_from_python_stage1_data* data) {
        auto xptr = PyTuple_GetItem(obj_ptr, 0);
        auto x = PyFloat_AsDouble(xptr);
        auto yptr = PyTuple_GetItem(obj_ptr, 1);
        auto y = PyFloat_AsDouble(yptr);
        void* storage = ((python::converter::rvalue_from_python_storage<Vec>*) data)->storage.bytes;
        new (storage) Vec{x, y};
        data->convertible = storage;
    }
};

int vec_len(const Vec& /*v*/) {
    return 2;
}

double vec_getitem(const Vec& v, int index) {
    if (index == 0 || index == -2) {
        return v.x;
    }
    if (index == 1 || index == -1) {
        return v.y;
    }
    PyErr_SetString(PyExc_IndexError, "Vec index out of range");
    python::throw_error_already_set();
    return -1; // Never reached LCOV_EXCL_LINE
}

void vec_setitem(Vec& v, int index, double value) {
    if (index == 0 || index == -2) {
        v.x = value;
    }
    else if (index == 1 || index == -1) {
        v.y = value;
    }
    else {
        PyErr_SetString(PyExc_IndexError, "Vec index out of range");
        python::throw_error_already_set();
    }
}

string vec_repr(const Vec& v) {
    stringstream ss;
    ss << "Vec" << v;
    return ss.str();
}

python::tuple vec_as_tuple(const Vec& v) {
    return python::make_tuple(v.x, v.y);
}

void wrap_vec() {
    Vec_from_tuple();
    // Note: inplace operators are not wrapped so vectors are immutable in python. As python does not copy objects, this makes everything safer
    python::class_<Vec>("Vec", python::init<double, double>())
        .def(python::init<>())
        .def(python::init<const Vec&>())
        .def_readonly("x", &Vec::x)
        .def_readonly("y", &Vec::y)
        .def(-op::self)
        .def(op::self == op::self)
        .def(op::self != op::self)
        .def(op::self < op::self)
        .def(op::self <= op::self)
        .def(op::self > op::self)
        .def(op::self >= op::self)
        .def("__bool__", &Vec::operator bool)
        .def("__abs__", &Vec::abs)
        .def("abs", &Vec::abs)
        .def("bearing", &Vec::bearing)
        .def("distance_to", &Vec::distanceTo)
        .def("bearing_to", &Vec::bearingTo)
        .def("cross", &Vec::cross)
        .def("dot", &Vec::dot)
        .def("squared", &Vec::squared)
        .def("norm", &Vec::norm)
        .def("perp", &Vec::perp)
        .def("proj", &Vec::proj)
        .def("rej", &Vec::rej)
        .def("from_bearing", &Vec::fromBearing)
        .staticmethod("from_bearing")
        .def("rotate", &Vec::rotate)
        .def(op::self + python::other<Vec>())
        .def(op::self - python::other<Vec>())
        .def(python::other<Vec>() + op::self)
        .def(python::other<Vec>() - op::self)
        .def(op::self * python::other<double>())
        .def(python::other<double>() * op::self)
        .def(op::self / python::other<double>())
        .def(op::str(op::self))
        .def("__repr__", vec_repr)
        .def("__len__", vec_len)
        .def("__getitem__", vec_getitem)
        .def("__setitem__", vec_setitem)
        .def("as_tuple", vec_as_tuple);
}
