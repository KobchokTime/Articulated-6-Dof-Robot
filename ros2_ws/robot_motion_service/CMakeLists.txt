cmake_minimum_required(VERSION 3.8)
project(robot_motion_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add custom service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPosition.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Ensure the generated headers are available for dependent packages
ament_export_dependencies(rosidl_default_runtime)

# Install Python scripts
install(PROGRAMS
  scripts/motion_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Enable linting if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()

