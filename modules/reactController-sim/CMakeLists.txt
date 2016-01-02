# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(reactController-sim)

find_package(IPOPT REQUIRED)

include_directories(${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}                    
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin ${IPOPT_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

