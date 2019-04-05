cmake_minimum_required(VERSION 2.8.3)
project(navi_service)	

SET(SOURCES src/navi_service_node.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  message_generation
  control_msgs
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
  actionlib_msgs
  actionlib
)

#find_package(Eigen3 REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(PCL 1.7 REQUIRED)

add_service_files(
   FILES
   GoTargetPos.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Approach.action
  ObsChecker.action
  BaseSlide.action
)


generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   std_srvs
   geometry_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES humantracking
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  tf
  actionlib
  actionlib_msgs
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_executable(nav_service_test src/service_test.cpp ${SOURCES})
target_link_libraries(nav_service_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(nav_service_test ${catkin_EXPORTED_TARGETS})

add_executable(obscheck_server src/obscheck_server.cpp ${SOURCES})
target_link_libraries(obscheck_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(obscheck_server ${catkin_EXPORTED_TARGETS})


#add_executable(obstacle_checker src/obstacle_checker.cpp ${SOURCES})
#target_link_libraries(obstacle_checker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(obstacle_checker ${catkin_EXPORTED_TARGETS})

#add_executable(map_checker src/map_checker.cpp ${SOURCES})
#target_link_libraries(map_checker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(map_checker ${catkin_EXPORTED_TARGETS})


#add_executable(clicked_navi_test src/clicked_navi_test.cpp ${SOURCES})
#target_link_libraries(clicked_navi_test ${catkin_LIBRARIES})
