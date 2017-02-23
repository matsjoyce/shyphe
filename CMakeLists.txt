cmake_minimum_required (VERSION 3.6)
project(physics)

find_package(ECM 0.0.11 REQUIRED NO_MODULE)
set(CMAKE_MODULE_PATH ${ECM_MODULE_PATH} ${ECM_KDE_MODULE_DIR})

find_package(PythonLibs 3 REQUIRED)
find_program(PYTHON "python3" REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(Boost COMPONENTS python3 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

include(CheckCXXCompilerFlag)

# http://stackoverflow.com/a/33266748/3946766
function(enable_cxx_compiler_flag_if_supported flag)
    string(FIND "${CMAKE_CXX_FLAGS}" "${flag}" flag_already_set)
    if(flag_already_set EQUAL -1)
        check_cxx_compiler_flag("${flag}" flag_supported)
        if(flag_supported)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}" PARENT_SCOPE)
        endif()
    endif()
endfunction()

set(CMAKE_CXX_STANDARD 11)

enable_cxx_compiler_flag_if_supported("-Wall")
enable_cxx_compiler_flag_if_supported("-Wextra")
enable_cxx_compiler_flag_if_supported("-pedantic")

execute_process(COMMAND ${PYTHON}-config --ldflags
                    OUTPUT_VARIABLE PYTHON_LDFLAGS
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
string(STRIP "${PYTHON_LDFLAGS}" PYTHON_LDFLAGS)

include_directories(src)

add_library(physics SHARED
            src/aabb.cpp src/body.cpp src/circle.cpp
            src/collider.cpp src/collisions.cpp src/sataxes.cpp
            src/shape.cpp src/vec.cpp src/python/module.cpp)
set_target_properties(physics PROPERTIES PREFIX "")
target_link_libraries(physics ${Boost_LIBRARIES})
target_link_libraries(physics ${PYTHON_LDFLAGS})

add_executable(test_physics test.cpp)
target_link_libraries(test_physics physics)

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
