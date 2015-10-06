# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <alessandro.roncone@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(reactController)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${source} ${header} )

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} iKin)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

