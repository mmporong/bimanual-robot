cmake_minimum_required(VERSION 3.16)
project(hold_flow_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes third_party urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
