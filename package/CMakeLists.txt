cmake_minimum_required(VERSION 2.8.3)
project(localization_with_simple_ekf)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(ATTENTION_TO_QR_CODE_SOURCE_DIR
  src/source
)

set(ATTENTION_TO_QR_CODE_INCLUDE_DIR
  src/include
)

set(ATTENTION_TO_QR_CODE_HEADER_FILES
        ${ATTENTION_TO_QR_CODE_INCLUDE_DIR}/*.h
)

set(ATTENTION_TO_QR_CODE_SOURCE_FILES
        ${ATTENTION_TO_QR_CODE_SOURCE_DIR}/*.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  behavior_execution_controller
  behavior_execution_manager_msgs
  geometry_msgs
  droneMsgsROS
  tf
  tf_conversions
  lib_pugixml
  lib_cvgutils
  bebop_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${ATTENTION_TO_QR_CODE_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
  std_msgs
  behavior_execution_controller
  behavior_execution_manager_msgs
  geometry_msgs
  droneMsgsROS
  tf
  tf_conversions
  lib_pugixml
  lib_cvgutils
  bebop_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(localization_with_simple_ekf
  src/source/behavior_localization_with_simple_ekf.cpp
)
target_link_libraries(localization_with_simple_ekf ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(localization_with_simple_ekf ${catkin_EXPORTED_LIBRARIES})
endif()
