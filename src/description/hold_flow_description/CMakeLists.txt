cmake_minimum_required(VERSION 3.16)
project(hold_flow_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes third_party urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/audit_urdf_quality.py
    scripts/import_isaac_sim.py
    scripts/validate_description.py
    scripts/validate_isaac_contract.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
