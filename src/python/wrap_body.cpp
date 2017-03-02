#include <boost/python.hpp>
#include <map>

#include "module.hpp"
#include "body.hpp"
#include "shape.hpp"
#include "sensor.hpp"
#include "circle.hpp"
#include "massshape.hpp"

using namespace std;
namespace python = boost::python;
namespace op = boost::python::self_ns;

class BodyWrap: public Body {
    PyObject *self;
    map<Shape*, python::object> pyshapes;
    map<Sensor*, python::object> pysensor;
public:
    BodyWrap(PyObject *p, const Vec& position_/*={}*/, const Vec& velocity_/*={}*/, const Vec& acceleration_/*={}*/,
             double angle_/*=0*/, double angular_velocity_/*=0*/, double angular_acceleration_/*=0*/,
             int side_/*=0*/): Body(position_, velocity_, acceleration_, angle_, angular_velocity_, angular_acceleration_, side_), self(p) {
    }

    python::object get_python_object() {
        return python::object(python::handle<>(python::borrowed(self)));
    }

    void add_shape(python::object shape) {
        auto ptr = python::extract<Shape*>(shape);
        pyshapes[ptr] = shape;
        addShape(ptr);
    }

    void remove_shape(python::object shape) {
        auto ptr = python::extract<Shape*>(shape);
        pyshapes.erase(ptr);
        removeShape(ptr);
    }

    void add_sensor(python::object sensor) {
        auto ptr = python::extract<Sensor*>(sensor);
        pysensor[ptr] = sensor;
        addSensor(ptr);
    }

    void remove_sensor(python::object sensor) {
        auto ptr = python::extract<Sensor*>(sensor);
        pysensor.erase(ptr);
        removeSensor(ptr);
    }
};

python::object magic_body_extract(Body* body) {
    if (!body) {
        return python::object(); // None
    }
    auto bw = dynamic_cast<BodyWrap*>(body);
    if (!bw) {
        throw runtime_error("Request to get_body failed, was this body created from Python?"); // LCOV_EXCL_LINE
    }
    return bw->get_python_object();
}

void wrap_body() {
    // Note: All the Vec properties have to return copies to preserve immutability
    python::class_<Body, boost::noncopyable, boost::shared_ptr<BodyWrap>>("Body",
        python::init<const Vec&, const Vec&, const Vec&, double, double, double, int>((python::arg("position")=Vec{},
                                                                                       python::arg("velocity")=Vec{},
                                                                                       python::arg("acceleration")=Vec{},
                                                                                       python::arg("angle")=0,
                                                                                       python::arg("angular_velocity")=0,
                                                                                       python::arg("angular_acceleration")=0,
                                                                                       python::arg("side")=0)))
        .add_property("position",
            python::make_getter(&Body::position, python::return_value_policy<python::return_by_value>()),
            python::make_setter(&Body::position)
        )
        .add_property("velocity",
            python::make_getter(&Body::velocity, python::return_value_policy<python::return_by_value>()),
            python::make_setter(&Body::velocity)
        )
        .add_property("acceleration",
            python::make_getter(&Body::acceleration, python::return_value_policy<python::return_by_value>()),
            python::make_setter(&Body::acceleration)
        )
        .def_readwrite("angle", &Body::angle)
        .def_readwrite("angular_velocity", &Body::angular_velocity)
        .def_readwrite("angular_acceleration", &Body::angular_acceleration)
        .def_readwrite("side", &Body::side)
        .def_readwrite("sensor_view", &Body::sensor_view)
        .add_property("mass", &Body::mass)
        .add_property("max_sensor_range", &Body::maxSensorRange)
        .def("update_position", &Body::updatePosition)
        .def("update_velocity", &Body::updateVelocity)
        .def("apply_impulse", &Body::applyImpulse)
        .def("add_shape", &BodyWrap::add_shape)
        .def("remove_shape", &BodyWrap::remove_shape)
        .def("add_sensor", &BodyWrap::add_sensor)
        .def("remove_sensor", &BodyWrap::remove_sensor);
    python::class_<Signature>("Signature")
        .def_readwrite("radar_emissions", &Signature::radar_emissions)
        .def_readwrite("thermal_emissions", &Signature::thermal_emissions)
        .def_readwrite("radar_cross_section", &Signature::radar_cross_section);
    python::class_<Shape, boost::noncopyable>("Shape", python::no_init)
        .def_readwrite("mass", &Shape::mass)
        .def_readwrite("signature", &Shape::signature)
        .def("clone", &Shape::clone, python::return_value_policy<python::manage_new_object>());
    python::class_<Circle, boost::noncopyable, python::bases<Shape>>("Circle",
        python::init<double, double, const Vec&, double, double, double>((python::arg("radius")=0,
                                                                          python::arg("mass")=0,
                                                                          python::arg("position")=Vec{},
                                                                          python::arg("radar_cross_section")=0,
                                                                          python::arg("radar_emissions")=0,
                                                                          python::arg("thermal_emissions")=0)))
        .def_readwrite("radius", &Circle::radius);
    python::class_<MassShape, boost::noncopyable, python::bases<Shape>>("MassShape",
        python::init<double, const Vec&, double, double, double>((python::arg("mass")=0,
                                                                  python::arg("position")=Vec{},
                                                                  python::arg("radar_cross_section")=0,
                                                                  python::arg("radar_emissions")=0,
                                                                  python::arg("thermal_emissions")=0)));
    IterableConverter<vector<SensedObject>>();
}
