cmake_minimum_required (VERSION 3.6)
project(shyphe)

set(PROJECT_FILES src/aabb.cpp src/body.cpp src/circle.cpp
                  src/collisions.cpp src/massshape.cpp src/polygon.cpp
                  src/sataxes.cpp src/sensor.cpp src/shape.cpp src/vec.cpp
                  src/world.cpp src/python/module.cpp src/python/wrap_body.cpp
                  src/python/wrap_collisions.cpp src/python/wrap_sensors.cpp
                  src/python/wrap_vec.cpp src/python/wrap_world.cpp)

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

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-z,defs")
enable_cxx_compiler_flag_if_supported("-Wall")
enable_cxx_compiler_flag_if_supported("-Wextra")
enable_cxx_compiler_flag_if_supported("-pedantic")
enable_cxx_compiler_flag_if_supported("-fdiagnostics-color=always")

execute_process(COMMAND ${PYTHON}-config --ldflags
                    OUTPUT_VARIABLE PYTHON_LDFLAGS
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
string(STRIP "${PYTHON_LDFLAGS}" PYTHON_LDFLAGS)

include_directories(src)

add_library(shyphe SHARED ${PROJECT_FILES})
set_target_properties(shyphe PROPERTIES PREFIX "")
target_link_libraries(shyphe ${Boost_LIBRARIES})
target_link_libraries(shyphe ${PYTHON_LDFLAGS})

add_library(shyphe_coverage SHARED ${PROJECT_FILES} src/python/coverage.cpp)
target_link_libraries(shyphe_coverage ${Boost_LIBRARIES})
target_link_libraries(shyphe_coverage ${PYTHON_LDFLAGS})
target_compile_options(shyphe_coverage PRIVATE "-fprofile-arcs"
                                                "-ftest-coverage"
                                                "-fno-elide-constructors"
                                                "-fno-inline"
                                                "-O0"
                                                "-g"
                                                "-fPIC"
                                                "-fno-default-inline"
                                                "-DCOVERAGE"
                                                "-lgcov")
set_target_properties(shyphe_coverage PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/coverage"
                                                  OUTPUT_NAME "shyphe"
                                                  PREFIX "")
target_link_libraries(shyphe_coverage gcov)

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
