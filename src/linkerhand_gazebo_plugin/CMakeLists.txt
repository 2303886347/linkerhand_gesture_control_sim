cmake_minimum_required(VERSION 3.8)
project(linkerhand_gazebo_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)

add_library(linkerhand_online_joint_controller SHARED
  src/online_joint_controller.cpp
)

target_link_libraries(linkerhand_online_joint_controller
  ignition-gazebo6::core
  ignition-msgs8::core
  ignition-plugin1::register
  ignition-transport11::core
)

install(TARGETS linkerhand_online_joint_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/linkerhand_gazebo_plugin.dsv"
)

ament_package()
