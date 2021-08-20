cmake_minimum_required(VERSION 3.8)
project(trace_samples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(end_to_end
  src/end_to_end.cpp)
ament_target_dependencies(end_to_end rclcpp sensor_msgs)
install(TARGETS
  end_to_end
  DESTINATION lib/${PROJECT_NAME})

add_executable(talker_listener
  src/talker_listener.cpp)
ament_target_dependencies(talker_listener rclcpp sensor_msgs)
install(TARGETS
  talker_listener
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
