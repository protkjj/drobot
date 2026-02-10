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

# 폴더(DIRECTORY) 단위로 통째로 복사하는 설정입니다.
install(DIRECTORY
  urdf
  meshes
  # launch  <-- 만약 description 패키지 안에 런치 폴더를 만들었다면 주석을 해제하세요.
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
