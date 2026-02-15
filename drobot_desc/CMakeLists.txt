cmake_minimum_required(VERSION 3.8)
project(drobot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf meshes launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
