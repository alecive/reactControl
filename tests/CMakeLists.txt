# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(tests)

find_package(IPOPT REQUIRED)

include_directories(${IPOPT_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)
add_executable(test-reactController-position-direct test-reactController-position-direct.cpp)
target_link_libraries(test-reactController-position-direct ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})
install(TARGETS test-reactController-position-direct DESTINATION bin)

