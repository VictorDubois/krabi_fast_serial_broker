cmake_minimum_required(VERSION 3.5)
project(krabi_fast_serial_broker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(krabi_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)


add_executable(${PROJECT_NAME}_node
  src/simple_stm32_broker.cpp)

ament_target_dependencies(${PROJECT_NAME}_node std_msgs geometry_msgs tf2 sensor_msgs krabi_msgs nav_msgs
  rclcpp)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()