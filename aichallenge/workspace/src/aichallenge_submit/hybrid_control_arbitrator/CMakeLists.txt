cmake_minimum_required(VERSION 3.8)
project(hybrid_control_arbitrator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install entry point script for the node
install(PROGRAMS
  hybrid_control_arbitrator/hybrid_control_arbitrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files to share
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
