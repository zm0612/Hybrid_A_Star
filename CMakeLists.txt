cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star)

message(STATUS "Hybrid A star")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf
    nav_core
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES hybrid_a_star 
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)

add_library(
    ${PROJECT_NAME}
    src/rs_path.cpp
    src/costmap_subscriber.cpp
    src/goal_pose_subscriber.cpp
    src/init_pose_subscriber.cpp
    src/hybrid_a_star.cpp
    src/hybrid_a_star_flow.cpp
)

add_library(hybrid_astar_global_planner
   src/hybrid_astar_global_planner.cpp)
   target_link_libraries(hybrid_astar_global_planner
   ${catkin_LIBRARIES} hybrid_a_star
)

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
)

add_executable(run_hybrid_astar app/run_hybrid_astar.cpp)
target_link_libraries(run_hybrid_astar ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES bgp_plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)