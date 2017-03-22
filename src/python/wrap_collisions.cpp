#include <boost/python.hpp>

#include "module.hpp"
#include "collisions.hpp"
#include "circle.hpp"
#include "polygon.hpp"
#include "world.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;


python::object get_collision_body(const Collision& col) {
    return magic_body_extract(col.body);
}

python::object get_collision_other(const Collision& col) {
    return magic_body_extract(col.other);
}

python::object get_collisiontime_a(const CollisionTimeResult& col) {
    return magic_body_extract(col.a);
}

python::object get_collisiontime_b(const CollisionTimeResult& col) {
    return magic_body_extract(col.b);
}

void wrap_collisions() {
    python::class_<Collision>("Collision")
        .add_property("body", &get_collision_body)
        .add_property("other", &get_collision_other)
        .def_readonly("time", &Collision::time)
        .def_readonly("touch_point", &Collision::touch_point)
        .def_readonly("impulse", &Collision::impulse)
        .def_readonly("closing_velocity", &Collision::closing_velocity)
        .def("apply_impulse", &Collision::apply_impulse);
    py_pair<Collision, Collision>();
    python::def("collide_shapes", collideShapes);
    python::def("distance_between", distanceBetween);
    python::def("collision_result", collisionResult);
    python::class_<DistanceResult>("CollisionTimeResult", python::init<double, Vec, Vec, Vec>())
        .def_readonly("distance", &DistanceResult::distance)
        .def_readonly("a_point", &DistanceResult::a_point)
        .def_readonly("b_point", &DistanceResult::b_point)
        .def_readonly("normal", &DistanceResult::normal);
    python::class_<CollisionTimeResult>("CollisionTimeResult", python::init<Body*, Body*, double, Vec, Vec>())
        .add_property("a", &get_collisiontime_a)
        .add_property("b", &get_collisiontime_b)
        .def_readonly("time", &CollisionTimeResult::time)
        .def_readonly("touch_point", &CollisionTimeResult::touch_point)
        .def_readonly("normal", &CollisionTimeResult::normal);
    python::class_<CollisionResult>("CollisionResult")
        .def_readonly("impulse", &CollisionResult::impulse)
        .def_readonly("closing_velocity", &CollisionResult::closing_velocity);
    python::class_<CollisionParameters>("CollisionParameters", python::init<double, double, double>())
        .def_readwrite("restitution", &CollisionParameters::restitution)
        .def_readwrite("transition_impulse", &CollisionParameters::transition_impulse)
        .def_readwrite("transition_reduction", &CollisionParameters::transition_reduction);
}
