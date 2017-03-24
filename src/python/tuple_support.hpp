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

#ifndef PYTHON_TUPLE_SUPPORT_HPP
#define PYTHON_TUPLE_SUPPORT_HPP

#include <tuple>
#include <utility>
#include <boost/python.hpp>

namespace python = boost::python;

template<class... Types> struct TupleToPythonConverter {
    static PyObject* convert(const std::tuple<Types...>& tuple) {
        return python::incref(helper(std::make_index_sequence<sizeof...(Types)>(), tuple).ptr());
    }

    template <size_t... Indexes> static python::tuple helper(std::index_sequence<Indexes...>, const std::tuple<Types...>& tuple) {
        return python::make_tuple(std::get<Indexes>(tuple)...);
    }
};

template<class... Types> struct PythonToTupleConverter {
    PythonToTupleConverter() {
        python::converter::registry::push_back(&convertible, &construct, python::type_id<std::tuple<Types...>>());
    }

    static void* convertible(PyObject* obj) {
        if (!PyTuple_CheckExact(obj)) return nullptr;
        if (PyTuple_Size(obj) != sizeof...(Types)) return nullptr;
        return obj;
    }

    static void construct(PyObject* obj, python::converter::rvalue_from_python_stage1_data* data) {
        python::tuple tuple(python::borrowed(obj));
        void* storage = ((python::converter::rvalue_from_python_storage<std::tuple<Types...>>*) data)->storage.bytes;
        new (storage) std::tuple<Types...>(helper(std::make_index_sequence<sizeof...(Types)>(), tuple));
        data->convertible = storage;
    }

    template<int N> using nth_type = typename std::tuple_element<N, std::tuple<Types...>>::type;

    template <size_t... Indexes> static std::tuple<Types...> helper(std::index_sequence<Indexes...>, python::tuple tuple) {
        return std::make_tuple(static_cast<nth_type<Indexes>>(python::extract<nth_type<Indexes>>(tuple[Indexes]))...);
    }
};

template<class... Types> struct TupleConverter {
    python::to_python_converter<std::tuple<Types...>, TupleToPythonConverter<Types...>> toPy;
    PythonToTupleConverter<Types...> fromPy;
};

#endif // PYTHON_TUPLE_SUPPORT_HPP
