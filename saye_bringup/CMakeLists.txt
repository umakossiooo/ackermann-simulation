cmake_minimum_required(VERSION 3.5)

project(saye_bringup)

find_package(ament_cmake REQUIRED)
find_package(saye_description REQUIRED)

install(
  DIRECTORY
    launch
    maps
    rviz
    config
    behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/save_map.py
  scripts/path_planning/dijkstra_path_planner.py
  scripts/path_planning/astar_path_planner.py
  scripts/path_planning/base_path_planner.py
  scripts/path_planning/base_path_planner_node.py
  scripts/path_planning/path_planner_utils.py
  DESTINATION lib/${PROJECT_NAME}/path_planning
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
