cmake_minimum_required(VERSION 3.8)
project(linkerhand_l30_right_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
