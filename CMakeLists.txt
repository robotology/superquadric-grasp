# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Giulia Vezzani <giulia.vezzani@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

project(superquadric-grasp)

find_package(YARP COMPONENTS OS sig dev math gsl)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpIDL)
include(YarpInstallationHelpers)

find_package(OpenCV REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

find_package(IPOPT REQUIRED)

option(USE_FINGERS_POSITION_CONTROL "Use FingersPositionControl instead of TactileControl" ON)
if(USE_FINGERS_POSITION_CONTROL)
  set(fingers_library FingersPositionControl)
  add_definitions(-DUSE_FINGERS_POSITION_CONTROL)
else()
  set(fingers_library TactileControl)
endif()

find_package(${fingers_library} REQUIRED)

set(sources src/superquadric.cpp src/main.cpp src/graspModule.cpp src/graspComputation.cpp src/graspVisualization.cpp src/graspExecution.cpp)
set(headers include/superquadric.h include/graspModule.h include/graspComputation.h include/graspVisualization.h include/graspExecution.h)
yarp_add_idl(IDL_GEN_FILES src/idl.thrift)
source_group("IDL Files" FILES src/idl.thrift)
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories( ${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS} iKin)

add_definitions(${YARP_DEFINES} ${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES) 

add_executable(${PROJECT_NAME} ${sources} ${headers} src/idl.thrift  ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${IPOPT_LIBRARIES} iKin ctrlLib ${fingers_library}::${fingers_library})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
icubcontrib_add_uninstall_target()


# app and conf
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
#yarp_install(FILES ${PROJECT_NAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
