cmake_minimum_required(VERSION 3.8)
project(drobot_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  worlds
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/generate_world.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
