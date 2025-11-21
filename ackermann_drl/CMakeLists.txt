cmake_minimum_required(VERSION 3.5)
project(ackermann_drl)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install Python package
# Install the main package directory (ackermann_drl/ackermann_drl/)
ament_python_install_package(${PROJECT_NAME})

# Install Python subdirectories (envs/, utils/) that are at package root
# These need to be installed as part of the package
install(DIRECTORY
  envs
  utils
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/train_ppo.py
  scripts/eval_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install test files
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Note: Test scripts are in the test/ directory at workspace root
  # They can be run directly without installation
endif()

ament_package()

