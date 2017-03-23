#ifndef PYTHON_CONTAINER_SUPPORT_HPP
#define PYTHON_CONTAINER_SUPPORT_HPP

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace python = boost::python;

template <typename Container, bool noproxy=false> struct ContainerConverter {
  ContainerConverter(const char* name) {
    python::converter::registry::push_back(&convertible, &construct, python::type_id<Container>());
    python::class_<Container>(name)
        .def(python::vector_indexing_suite<Container, noproxy>());
  }

  static void* convertible(PyObject* obj) {
    return PyObject_GetIter(obj) ? obj : nullptr;
  }

  static void construct(PyObject* obj, python::converter::rvalue_from_python_stage1_data* data) {
        python::handle<> handle(python::borrowed(obj));
        void* storage = ((python::converter::rvalue_from_python_storage<Container>*) data)->storage.bytes;
        typedef python::stl_input_iterator<typename Container::value_type> iterator;
        new (storage) Container(iterator(python::object(handle)), iterator());
        data->convertible = storage;
    }
};

#endif // PYTHON_CONTAINER_SUPPORT_HPP
