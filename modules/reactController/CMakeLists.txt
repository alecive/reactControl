# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Alessandro Roncone <alessandro.roncone@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(reactController)

find_package(IPOPT REQUIRED)

include(YarpIDL)

set(header_files    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}_IDL.h
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/reactIpOpt.h
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/reactCtrlThread.h
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/particleThread.h)
set(source_files    ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_IDL.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/reactIpOpt.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/reactCtrlThread.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/particleThread.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/reactController.cpp)
set(idl_files       ${PROJECT_NAME}.thrift)

yarp_idl_to_dir(${idl_files} ${CMAKE_CURRENT_SOURCE_DIR})

source_group("Header Files" FILES ${header_files})
source_group("Source Files" FILES ${source_files})
source_group("IDL Files"    FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${IPOPT_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}                    
                    ${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} ${header_files} ${source_files} ${idl_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib iKin skinDynLib ${IPOPT_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

