# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(reactController-sim)

find_package(IPOPT REQUIRED)

include_directories(${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}                    
                    ${ICUB_INCLUDE_DIRS})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

