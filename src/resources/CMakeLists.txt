cmake_minimum_required(VERSION 3.20)
project(resources)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    cfg
    urdf
    meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
