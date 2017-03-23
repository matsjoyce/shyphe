#ifndef PYTHON_SHARED_PTR_SUPPORT_HPP
#define PYTHON_SHARED_PTR_SUPPORT_HPP

#include <map>
#include <boost/python.hpp>

namespace python = boost::python;

template <class T> void update_cache(T*, PyObject*);

template<typename T> class py_wrap : public T {
public:
    template <class... Args> py_wrap(PyObject* p, Args... args) : T(args...) {
        update_cache<T>(this, p);
    }
};

template<typename T> class py_ptr {
public:
    typedef py_wrap<T> element_type;

    py_ptr(element_type* t) : ptr(t) {
        std::get<1>(cache()[get()]) += 1;
    }

    py_ptr(const std::shared_ptr<T>& t) : ptr(t) {
        std::get<1>(cache()[get()]) += 1;
    }

    py_ptr(const py_ptr<T>& t) : ptr(t.ptr) {
        std::get<1>(cache()[get()]) += 1;
    }

    ~py_ptr() {
        T* key = get();
        std::get<1>(cache()[key]) -= 1;
        if (!std::get<1>(cache()[key])) {
            cache().erase(key);
        }
    }

    static std::map<T*, std::tuple<PyObject*, int>>& cache() {
        static std::map<T*, std::tuple<PyObject*, int>> c;
        return c;
    }

    static PyObject* get_or_create(const std::shared_ptr<T>& obj) {
        auto key = obj.get();
        if (cache().count(key)) {
            return python::incref(std::get<0>(cache()[key]));
        }
        PyObject* ptr = python::incref(python::objects::make_ptr_instance<T, python::objects::pointer_holder<py_ptr<T>, T>>::execute(obj));
        std::get<0>(cache()[key]) = ptr;
        return ptr;
    }

    T* get() const {
        return ptr.get();
    }

private:
    std::shared_ptr<T> ptr;
};

template<typename T> typename py_ptr<T>::element_type* get_pointer(const py_ptr<T>& pp) {
    return static_cast<typename py_ptr<T>::element_type*>(pp.get());
}

template <class T> void update_cache(T* key, PyObject* ptr) {
    std::get<0>(py_ptr<T>::cache()[key]) = ptr;
}

template<typename T> struct SharedToPythonConverter {
    static PyObject* convert(const std::shared_ptr<T>& obj) {
        return py_ptr<T>::get_or_create(obj);
    }
};

template<typename T> struct SharedConverter {
    python::to_python_converter<std::shared_ptr<T>, SharedToPythonConverter<T>> toPy;
};

#endif // PYTHON_SHARED_PTR_SUPPORT_HPP

