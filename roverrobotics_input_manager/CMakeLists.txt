cmake_minimum_required(VERSION 3.5)
project(roverrobotics_input_manager)

find_package(ament_cmake REQUIRED)


install(DIRECTORY
  scripts/modules
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/joys_manager.py
  DESTINATION lib/${PROJECT_NAME})


ament_package()
