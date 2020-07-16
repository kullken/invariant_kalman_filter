cmake_minimum_required(VERSION 3.10.2)
project(invariant_kalman_filter)

# add_compile_options(-std=c++17 -g -Wall -Wextra -Wpedantic -Wno-deprecated-copy)
add_compile_options(-std=c++17)

add_library(warnings INTERFACE)
target_compile_options(warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor 
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wnull-dereference
    -Wmisleading-indentation
    -Wduplicated-cond
    -Wduplicated-branches
    -Wno-deprecated-copy
)

find_package(catkin REQUIRED 
  COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
)

find_package(ugl REQUIRED)

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_iekf
  src/iekf.cpp
)

target_link_libraries(${PROJECT_NAME}_iekf
  ${catkin_LIBRARIES}
  ugl::math
  ugl::lie_group
  warnings
)

## Declare a C++ executable
add_executable(kalman_node src/kalman_node.cpp)

target_link_libraries(kalman_node
  ${PROJECT_NAME}_iekf
  ${catkin_LIBRARIES}
  warnings
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

#############
## Testing ##
#############

## Mock Trajectory Node
add_executable(mock_trajectory_node
  test/src/mock_trajectory_node.cpp
)

target_include_directories(mock_trajectory_node
  PRIVATE
    test/include
)

target_link_libraries(mock_trajectory_node
  PRIVATE
    ${catkin_LIBRARIES}
    ugl::trajectory
    warnings
)

## Sensor Models
add_library(sensor_models
  test/src/sensor_models/imu_sensor_model.cpp
  test/src/sensor_models/mocap_sensor_model.cpp
)

target_include_directories(sensor_models
  PUBLIC
    test/include/sensor_models
)

target_link_libraries(sensor_models
  PUBLIC
    ugl::math
    ugl::trajectory
    warnings
)

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/src/test_main.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_include_directories(${PROJECT_NAME}-test
    PRIVATE
      test/include  
  )

  target_sources(${PROJECT_NAME}-test
    PRIVATE
      test/src/accuracy_test.cpp
      test/src/accuracy_test_config.cpp
      test/src/test_trajectories.cpp
      test/src/test_filters.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
      ${PROJECT_NAME}_iekf
      sensor_models
      ugl::math
      ugl::trajectory
      warnings
  )
endif()
