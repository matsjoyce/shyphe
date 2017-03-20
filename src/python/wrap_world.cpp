#include <boost/python.hpp>

#include "module.hpp"
#include "world.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;


void wrap_world() {
    python::class_<World>("World", python::init<double>())
        .def("add_body", &World::addBody)
        .def("remove_body", &World::removeBody)
        .def("begin_frame", &World::beginFrame)
        .def("end_frame", &World::endFrame)
        .def("next_collision", &World::nextCollision)
        .def("calculate_collision", &World::calculateCollision)
        .def("finished_collision", &World::finishedCollision)
        .def("has_next_collision", &World::hasNextCollision);
}
