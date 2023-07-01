cmake_minimum_required(VERSION 3.8)
project(roverrobotics_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rviz launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
