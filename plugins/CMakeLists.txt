cmake_minimum_required(VERSION 3.0.2)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#chh add
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_library(force_plugin_loggerOneTopic SHARED force_plugin_loggerOneTopic.cc)
target_link_libraries(force_plugin_loggerOneTopic ${GAZEBO_libraries} ${roscpp_LIBRARIES})


# Build our plugin
add_library(force_plugin_loggerOneTopic_2uav SHARED force_plugin_loggerOneTopic_2uav.cc)
target_link_libraries(force_plugin_loggerOneTopic_2uav ${GAZEBO_libraries} ${roscpp_LIBRARIES})

