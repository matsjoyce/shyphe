#include "module.hpp"
#include "collisions.hpp"
#include "body.hpp"
#include "shape.hpp"

using namespace std;

void wrap_collisions() {
    python::def("collide_shapes", collideShapes);
    python::def("distance_between", distanceBetween);
    python::def("collision_result", collisionResult);
    python::class_<DistanceResult>("DistanceResult", python::init<double, Vec, Vec, Vec>())
        .def_readonly("distance", &DistanceResult::distance)
        .def_readonly("a_point", &DistanceResult::a_point)
        .def_readonly("b_point", &DistanceResult::b_point)
        .def_readonly("normal", &DistanceResult::normal);
    python::class_<CollisionResult>("CollisionResult")
        .def_readonly("impulse", &CollisionResult::impulse)
        .def_readonly("closing_velocity", &CollisionResult::closing_velocity);
    python::class_<CollisionTimeResult>("CollisionTimeResult", python::init<double, Vec, Vec>())
        .def_readonly("time", &CollisionTimeResult::time)
        .def_readonly("touch_point", &CollisionTimeResult::touch_point)
        .def_readonly("normal", &CollisionTimeResult::normal);
    python::class_<CollisionParameters>("CollisionParameters", python::init<double>())
        .def_readwrite("restitution", &CollisionParameters::restitution);
}
