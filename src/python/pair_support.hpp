/*
 * shyphe - Stiff HIgh velocity PHysics Engine
 * Copyright (C) 2017 Matthew Joyce matsjoyce@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef PYTHON_PAIR_SUPPORT_HPP
#define PYTHON_PAIR_SUPPORT_HPP

#include <utility>
#include <boost/python.hpp>

namespace python = boost::python;

// http://stackoverflow.com/a/41593748/3946766

template<typename T1, typename T2> struct PairToPythonConverter {
    static PyObject* convert(const std::pair<T1, T2>& pair) {
        return python::incref(python::make_tuple(pair.first, pair.second).ptr());
    }
};

template<typename T1, typename T2> struct PythonToPairConverter {
    PythonToPairConverter() {
        python::converter::registry::push_back(&convertible, &construct, python::type_id<std::pair<T1, T2>>());
    }

    static void* convertible(PyObject* obj) {
        if (!PyTuple_CheckExact(obj)) return nullptr;
        if (PyTuple_Size(obj) != 2) return nullptr;
        return obj;
    }

    static void construct(PyObject* obj, python::converter::rvalue_from_python_stage1_data* data) {
        python::tuple tuple(python::borrowed(obj));
        void* storage = ((python::converter::rvalue_from_python_storage<std::pair<T1, T2>>*) data)->storage.bytes;
        new (storage) std::pair<T1, T2>(python::extract<T1>(tuple[0]), python::extract<T2>(tuple[1]));
        data->convertible = storage;
    }
};

template<typename T1, typename T2> struct PairConverter {
    python::to_python_converter<std::pair<T1, T2>, PairToPythonConverter<T1, T2>> toPy;
    PythonToPairConverter<T1, T2> fromPy;
};

#endif // PYTHON_PAIR_SUPPORT_HPP
