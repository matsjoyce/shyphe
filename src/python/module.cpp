#include <string>
#include <sstream>
#include <boost/python.hpp>

#include "vec.hpp"
#include "body.hpp"

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
            PyErr_SetString(PyExc_ValueError, "Vec from tuple: index 0 is not a number");
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

string vec_repr(const Vec& v) {
    stringstream ss;
    ss << "Vec" << v;
    return ss.str();
}

BOOST_PYTHON_MODULE(physics) {
    Vec_from_tuple();
    python::class_<Vec>("Vec", python::init<double, double>())
        .def_readwrite("x", &Vec::x)
        .def_readwrite("y", &Vec::y)
        .def(-op::self)
        .def(op::self += op::self)
        .def(op::self -= op::self)
        .def(op::self *= python::other<double>())
        .def(op::self *= op::self)
        .def(op::self /= python::other<double>())
        .def(op::self /= op::self)
        .def(op::self == op::self)
        .def(op::self != op::self)
        .def(op::self < op::self)
        .def(op::self <= op::self)
        .def(op::self > op::self)
        .def(op::self >= op::self)
        .def("__abs__", &Vec::abs)
        .def("abs", &Vec::abs)
        .def("bearing", &Vec::bearing)
        .def("distance_to", &Vec::distance_to)
        .def("bearing_to", &Vec::bearing_to)
        .def("dot", &Vec::dot)
        .def("squared", &Vec::squared)
        .def("norm", &Vec::norm)
        .def("fromBearing", &Vec::fromBearing)
        .staticmethod("fromBearing")
        .def(op::self + op::self)
        .def(op::self - op::self)
        .def(op::self * python::other<double>())
        .def(python::other<double>() * op::self)
        .def(op::self * op::self)
        .def(op::self / python::other<double>())
        .def(op::self / op::self)
        .def(op::str(op::self))
        .def("__repr__", vec_repr)
        ;
    python::class_<Body>("Body")
        .def_readwrite("position", &Body::position)
        .def_readwrite("velocity", &Body::velocity)
        .def_readwrite("acceleration", &Body::acceleration)
        .def_readwrite("angle", &Body::angle)
        .def_readwrite("angular_velocity", &Body::angular_velocity)
        .def_readwrite("angular_acceleration", &Body::angular_acceleration)
        .def("updatePosition", &Body::updatePosition)
        .def("updateVelocity", &Body::updateVelocity)
        ;
}
