cmake_minimum_required(VERSION 3.8)
project(cpp_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
# This builds src/motor_control_node.cpp into an executable named "motor_control_node"
add_executable(motor_control_node src/motor_control_node.cpp)
add_executable(motor_control_node src/ps5.cpp)

# Link dependencies
ament_target_dependencies(motor_control_node
  "rclcpp"
  "geometry_msgs"
)

# Link the pthreads library (for std::thread and serial port)
target_link_libraries(motor_control_node pthread)

# Install the executable
install(TARGETS
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

