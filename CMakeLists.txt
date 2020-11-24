cmake_minimum_required(VERSION 2.8.3)
project(believer_dummy_nodes)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  mavros_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    geographic_msgs
    mavros_msgs
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dummy_offb_node src/dummy_offb_node.cpp)
target_link_libraries(dummy_offb_node ${catkin_LIBRARIES})
add_dependencies(dummy_offb_node ${catkin_EXPORTED_TARGETS})

install(TARGETS dummy_offb_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
