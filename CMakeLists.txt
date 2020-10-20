cmake_minimum_required(VERSION 3.0.2)
project(kuka_implementation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

find_package(catkin REQUIRED COMPONENTS
  autharl_core
  lwr_robot
  roscpp
  roslib
  sigproc_lib
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
#  LIBRARIES kuka_implementation
#  CATKIN_DEPENDS autharl_core/ roscpp
#  DEPENDS system_lib
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories("${BOOST_INCLUDE_DIRS}" )
message(${BOOST_LIBRARIES})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
# sources
include_directories(include ${Eigen3_INCLUDE_DIRS})

add_executable(main_collector src/main_collector.cpp src/collector.cpp )
target_link_libraries(main_collector ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${BOOST_LIBRARIES})
#add_dependencies(main_collector autharl_core)

add_executable(master src/master.cpp )
target_link_libraries(master ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${BOOST_LIBRARIES} ${autharl_core} sigproc_lib)
#add_dependencies(master sigproc_lib)

add_executable(master_smooth src/master_smooth.cpp )
target_link_libraries(master_smooth ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${BOOST_LIBRARIES} ${autharl_core} sigproc_lib)

add_executable(master_smooth_online src/master_smooth_online.cpp )
target_link_libraries(master_smooth_online ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${BOOST_LIBRARIES} ${autharl_core} sigproc_lib)
