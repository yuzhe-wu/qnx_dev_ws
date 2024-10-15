cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)  # Ensure yaml-cpp is found
find_package(chatter_interfaces REQUIRED)                      # CHANGE


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

link_directories(/home/wayve/yuzhe/ros2_qnx_workspace/ros2/install/aarch64le/lib)


add_executable(talker_colcon src/publisher_member_function.cpp)
# target_link_libraries(cpp_pubsub yaml-cpp)
# target_link_libraries(talker yaml-cpp)
ament_target_dependencies(talker_colcon rclcpp chatter_interfaces)

add_executable(listener_colcon src/subscriber_member_function.cpp)
ament_target_dependencies(listener_colcon rclcpp chatter_interfaces)


# link_directories(/home/wayve/yuzhe/ros2_qnx_workspace/ros2/install/aarch64le/lib)
# target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS
  talker_colcon
  listener_colcon
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
