cmake_minimum_required(VERSION 3.5)
project(mrs_uav_gazebo_testing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

set(CATKIN_DEPENDENCIES
  cmake_modules
  gazebo_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_testing
  roscpp
  rostest
  )

set(LIBRARIES
  MrsUavGazeboTesting_TestGeneric
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# Test Generic

add_library(MrsUavGazeboTesting_TestGeneric
  src/test_generic.cpp
  )

target_link_libraries(MrsUavGazeboTesting_TestGeneric
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_dependencies(MrsUavGazeboTesting_TestGeneric
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/mrs_uav_gazebo_testing/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
