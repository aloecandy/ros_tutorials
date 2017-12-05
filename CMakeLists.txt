cmake_minimum_required(VERSION 2.8.3)
project(pi)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  visualization_msgs
  sensor_msgs
)

add_service_files(FILES led.srv)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES pi
  CATKIN_DEPENDS message_generation message_runtime roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(led src/ws2812b.cpp src/led.cpp)
add_dependencies(led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(led ${catkin_LIBRARIES})

add_executable(led_client src/led_client.cpp)
add_dependencies(led_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(led_client ${catkin_LIBRARIES})

add_executable(ultrasonic src/ultrasonic.cpp)
add_dependencies(ultrasonic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultrasonic ${catkin_LIBRARIES})

add_executable(distance_to_led src/distance_to_led.cpp)
add_dependencies(distance_to_led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_to_led ${catkin_LIBRARIES})
