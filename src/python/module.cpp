#include <string>
#include <sstream>
#include <boost/python.hpp>

#include "utils.hpp"
#include "vec.hpp"
#include "body.hpp"
#include "shape.hpp"
#include "circle.hpp"
#include "collider.hpp"

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

string vec_repr(const Vec& v) {
    stringstream ss;
    ss << "Vec" << v;
    return ss.str();
}

class BodyWrap: public Body {
    PyObject *self;
public:
    BodyWrap(PyObject *p): self(p) {
    }

    python::object get_python_object() {
        return python::object(python::handle<>(python::borrowed(self)));
    }
};

python::object magic_body_extract(Body* body) {
    auto bw = dynamic_cast<BodyWrap*>(body);
    if (!bw) {
        throw runtime_error("Request to get_body failed, was this body created from Python?");
    }
    return bw->get_python_object();
}

python::object get_collision_body(const Collision& col) {
    return magic_body_extract(col.body);
}

python::object get_collision_other(const Collision& col) {
    return magic_body_extract(col.other);
}

class ShapeWrap: public Shape, public python::wrapper<Shape> {
    virtual AABB aabb() const override {
        throw runtime_error("Do not override shape!");
    }

    virtual CollisionTimeResult collide(const Shape* /*other*/, double /*end_time*/) const override {
        throw runtime_error("Do not override shape!");
    }

    virtual CollisionTimeResult collide(const Circle* /*other*/, double /*end_time*/) const  override {
        throw runtime_error("Do not override shape!");
    }
};

BOOST_PYTHON_MODULE(physics) {
    python::def("norm_rad", norm_rad);
    python::def("norm_deg", norm_deg);
    python::def("to_deg", to_deg);
    python::def("to_rad", to_rad);
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
        .def("__repr__", vec_repr);
    python::class_<Body, boost::noncopyable, boost::shared_ptr<BodyWrap>>("Body")
        .def_readwrite("position", &Body::position)
        .def_readwrite("velocity", &Body::velocity)
        .def_readwrite("acceleration", &Body::acceleration)
        .def_readwrite("angle", &Body::angle)
        .def_readwrite("angular_velocity", &Body::angular_velocity)
        .def_readwrite("angular_acceleration", &Body::angular_acceleration)
        .def("updatePosition", &Body::updatePosition)
        .def("updateVelocity", &Body::updateVelocity)
        .def("addShape", &Body::addShape);
    python::class_<ShapeWrap, boost::noncopyable>("Shape")
        .def_readwrite("mass", &Shape::mass);
    python::class_<Circle, boost::noncopyable, python::bases<Shape>>("Circle")
        .def_readwrite("radius", &Circle::radius);
    python::class_<Collider>("Collider")
        .def("newBody", &Collider::newBody)
        .def("reset", &Collider::reset)
        .def("nextCollision", &Collider::nextCollision)
        .def("finishedCollision", &Collider::finishedCollision)
        .def("hasNextCollision", &Collider::hasNextCollision);
    python::class_<Collision>("Collision")
        .add_property("body", &get_collision_body)
        .add_property("other", &get_collision_other)
        .def_readonly("time", &Collision::time);
    python::class_<pair<Collision, Collision> >("CollisionPair")
        .def_readonly("first", &pair<Collision, Collision>::first)
        .def_readonly("second", &pair<Collision, Collision>::second);
}
