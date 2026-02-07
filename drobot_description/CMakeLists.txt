cmake_minimum_required(VERSION 3.8)
project(drobot_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/world_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
