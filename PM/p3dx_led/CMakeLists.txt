cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo REQUIRED)

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(big_led SHARED big_led.cc)
target_link_libraries(big_led ${GAZEBO_LIBRARIES} ${roscppLIBRARIES} ${rosconsole_LIBRARIES})


#add_library(led_coloring SHARED led_coloring.cc)
#target_link_libraries(led_coloring ${GAZEBO_LIBRARIES} ${roscppLIBRARIES} ${rosconsole_LIBRARIES})

#only for test in model_coloring
#add_library(model_coloring SHARED model_coloring.cc)
#target_link_libraries(model_coloring ${GAZEBO_LIBRARIES})

## link to gazebo libraries 


