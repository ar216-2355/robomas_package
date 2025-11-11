cmake_minimum_required(VERSION 3.8)
project(robomas_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorFeedback.msg"
)

add_executable(talker src/receiver_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Use rosidl_get_typesupport_target + target_link_libraries instead of rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(talker "${cpp_typesupport_target}")

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()