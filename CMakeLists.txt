cmake_minimum_required(VERSION 2.8.3)
project(epic)

# Important: Must be C++11 since I assume in EpicNavigationNode that vector.h can be dereferenced to
# get the array.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav_core
  costmap_2d
  message_generation
)

add_service_files(
  FILES
  SetStatus.srv
  ModifyGoals.srv
  GetCell.srv
  SetCells.srv
  ResetFreeCells.srv
  ComputePath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES epic
  CATKIN_DEPENDS message_runtime
#  DEPENDS ompl #system_lib                     # STEP 2/5 FOR OMPL SUPPORT
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  libepic/include
#  ${OMPL_INCLUDE_DIRS}                         # STEP 3/5 FOR OMPL SUPPORT
)

link_directories(libepic/lib)

add_library(epic_nav_core_plugin
  src/epic_nav_core_plugin.cpp
)

add_executable(epic_navigation_node
  src/epic_navigation_node.cpp
  src/epic_navigation_node_harmonic.cpp
  src/epic_navigation_node_harmonic_rviz.cpp
#  src/epic_navigation_node_ompl.cpp            # STEP 4/5 FOR OMPL SUPPORT
  src/epic_navigation_node_main.cpp
)

add_dependencies(epic_nav_core_plugin epic_generate_messages_cpp)
add_dependencies(epic_navigation_node epic_generate_messages_cpp)

target_link_libraries(epic_nav_core_plugin
  ${catkin_LIBRARIES}
  epic
)

target_link_libraries(epic_navigation_node
  ${catkin_LIBRARIES}
  epic
#  ${OMPL_LIBRARIES}                            # STEP 5/5 FOR OMPL SUPPORT
)

