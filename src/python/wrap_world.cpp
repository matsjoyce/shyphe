#include "module.hpp"
#include "pair_support.hpp"
#include "container_support.hpp"
#include "world.hpp"

using namespace std;

void wrap_world() {
    python::class_<World>("World", python::init<double>())
        .def("add_body", &World::addBody)
        .def("remove_body", &World::removeBody)
        .def("begin_frame", &World::beginFrame)
        .def("end_frame", &World::endFrame)
        .def("next_collision", &World::nextCollision)
        .def("calculate_collision", &World::calculateCollision)
        .def("finished_collision", &World::finishedCollision)
        .def("has_next_collision", &World::hasNextCollision)
        .add_property("bodies", python::make_function(&World::bodies, python::return_internal_reference<>()));
    python::class_<UnresolvedCollision>("UnresolvedCollision", python::no_init)//, python::init<Body*, Body*, Shape*, Shape*, double, Vec, Vec>())
        .def_readonly("a", &UnresolvedCollision::a)
        .def_readonly("b", &UnresolvedCollision::b)
        .def_readonly("time", &UnresolvedCollision::time)
        .def_readonly("touch_point", &UnresolvedCollision::touch_point)
        .def_readonly("normal", &UnresolvedCollision::normal);
    python::class_<ResolvedCollision>("ResolvedCollision")
        .def_readonly("body", &ResolvedCollision::body)
        .def_readonly("other", &ResolvedCollision::other)
        .def_readonly("time", &ResolvedCollision::time)
        .def_readonly("touch_point", &ResolvedCollision::touch_point)
        .add_property("impulse",
             python::make_getter(&ResolvedCollision::impulse, python::return_value_policy<python::return_by_value>()),
             python::make_setter(&ResolvedCollision::impulse))
        .def_readonly("closing_velocity", &ResolvedCollision::closing_velocity)
        .def("apply_impulse", &ResolvedCollision::apply_impulse);
    PairConverter<ResolvedCollision, ResolvedCollision>();
    ContainerConverter<vector<shared_ptr<Body>>, true>("BodyVector");
}
