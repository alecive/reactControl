# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini, Matej Hoffmann
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME jointVelCtrlIdentSimple)
project(${PROJECTNAME})

include_directories(${IPOPT_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

