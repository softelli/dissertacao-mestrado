cmake_minimum_required(VERSION 2.8.3)
project(big_led_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
)
find_package (gazebo REQUIRED)

set(src 
    BigLedPlugin)

add_library(${src} SHARED ${src}.cc)
target_link_libraries(${src} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

# Build the stand-alone test program
#add_executable(setcor setcor.cc)

add_library(${PROJECT_NAME} ${src})





