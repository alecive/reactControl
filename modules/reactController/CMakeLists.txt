# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <alessandro.roncone@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(reactController)

find_package(IPOPT REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${IPOPT_INCLUDE_DIRS})

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(${PROJECT_NAME} ${source} ${header} )

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} iKin ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

