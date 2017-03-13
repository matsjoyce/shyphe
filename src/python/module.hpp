#ifndef PYTHON_MODULE_HPP
#define PYTHON_MODULE_HPP

#include <utility>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

void wrap_vec();
void wrap_collisions();
void wrap_body();
void wrap_sensors();
void wrap_world();

#ifdef COVERAGE
void wrap_coverage();
#endif

class Body;
boost::python::object magic_body_extract(Body* body);

// http://stackoverflow.com/a/41593748/3946766
template<typename T1, typename T2> struct PairToPythonConverter {
    static PyObject* convert(const std::pair<T1, T2>& pair) {
        return boost::python::incref(boost::python::make_tuple(pair.first, pair.second).ptr());
    }
};

template<typename T1, typename T2> struct PythonToPairConverter {
    PythonToPairConverter() {
        boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<std::pair<T1, T2>>());
    }

    static void* convertible(PyObject* obj) {
        if (!PyTuple_CheckExact(obj)) return nullptr;
        if (PyTuple_Size(obj) != 2) return nullptr;
        return obj;
    }

    static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data) {
        boost::python::tuple tuple(boost::python::borrowed(obj));
        void* storage = ((boost::python::converter::rvalue_from_python_storage<std::pair<T1, T2>>*) data)->storage.bytes;
        new (storage) std::pair<T1, T2>(boost::python::extract<T1>(tuple[0]), boost::python::extract<T2>(tuple[1]));
        data->convertible = storage;
    }
};

template<typename T1, typename T2> struct py_pair {
    boost::python::to_python_converter<std::pair<T1, T2>, PairToPythonConverter<T1, T2>> toPy;
    PythonToPairConverter<T1, T2> fromPy;
};

template <typename Container> struct IterableConverter {
  IterableConverter() {
    boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<Container>());
  }

  static void* convertible(PyObject* obj) {
    return PyObject_GetIter(obj) ? obj : nullptr;
  }

  static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data) {
        boost::python::handle<> handle(boost::python::borrowed(obj));
        void* storage = ((boost::python::converter::rvalue_from_python_storage<Container>*) data)->storage.bytes;
        typedef boost::python::stl_input_iterator<typename Container::value_type> iterator;
        new (storage) Container(iterator(boost::python::object(handle)), iterator());
        data->convertible = storage;
    }
};

#endif // PYTHON_MODULE_HPP
