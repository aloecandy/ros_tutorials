cmake_minimum_required(VERSION 2.8.3)
project(pi)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
)

#add_service_files(FILES SrvLed.srv)

#add_message_files(FILES MsgExample.msg)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  LIBRARIES pi
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fakesensor src/fakesensor.cpp)
add_dependencies(fakesensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fakesensor ${catkin_LIBRARIES})

