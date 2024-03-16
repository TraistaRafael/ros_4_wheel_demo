cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(control_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  sensor_msgs
  
  roslib
  geometry_msgs
  std_srvs
  tf
  rosgraph_msgs
  dynamic_reconfigure
)

# Find Gazebo
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()


# catkin specific configuration
catkin_package(CATKIN_DEPENDS message_runtime)


# Build our plugin
add_library(control_plugin SHARED control_plugin.cpp)
# target_link_libraries(control_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(control_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(control_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(control_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(control_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})