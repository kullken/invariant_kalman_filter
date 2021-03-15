cmake_minimum_required(VERSION 3.10.2)
project(invariant_kalman_filter)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
    ugl_ros
)

add_library(catkin_include_interface INTERFACE)
target_include_directories(catkin_include_interface SYSTEM INTERFACE ${catkin_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES catkin_include_interface)

find_package(ugl REQUIRED)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wno-deprecated-copy
)

include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)

enable_sanitizers(project_options)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#  LIBRARIES localisation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS
)

###########
## Build ##
###########

## Sensor Models ##
add_library(sensor_models SHARED
  src/kalman_filter/gps_model.cpp
  src/kalman_filter/imu_model.cpp
  src/kalman_filter/mocap_model.cpp
)
target_include_directories(sensor_models
  PUBLIC
    include/kalman_filter
)
target_link_libraries(sensor_models
  PUBLIC
    ugl::math
    ugl::lie_group
  PRIVATE
    project_options
    project_warnings
)

## Kalman filter ##
add_library(kalman_filter SHARED
  src/kalman_filter/iekf.cpp
  src/kalman_filter/mekf.cpp
  src/kalman_filter/mocap_model.cpp
  src/kalman_filter/gps_model.cpp
)
target_include_directories(kalman_filter
  PUBLIC
    include/kalman_filter
)
target_link_libraries(kalman_filter
  PUBLIC
    ugl::math
    ugl::lie_group
    sensor_models
  PRIVATE
    project_options
    project_warnings
)

## Kalman ROS-node ##
add_executable(kalman_node
  src/kalman_node/kalman_node.cpp
  src/kalman_node/measurement.cpp
)
target_include_directories(kalman_node
  PUBLIC
    include/kalman_node
  PRIVATE
    src/kalman_node
)
target_link_libraries(kalman_node
  PUBLIC
    kalman_filter
    ${catkin_LIBRARIES}
  PRIVATE
    ugl::math
    ugl::lie_group
    project_options
    project_warnings
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

add_subdirectory(test)
