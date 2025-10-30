cmake_minimum_required(VERSION 3.8)
project(saye_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  models
  worlds
  urdf
  hooks
  DESTINATION share/${PROJECT_NAME}/
)

# Optionally bundle external OSM-converted models into this package's models
# so model:// URIs resolve without extra env vars.
# These directories are outside this package tree but present in this repo.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../map_osm_converter/models/bari_3d
  ${CMAKE_CURRENT_SOURCE_DIR}/../../map_osm_converter/models/bari_2_3d
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL
)

ament_package()
