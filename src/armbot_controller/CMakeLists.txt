cmake_minimum_required(VERSION 3.8)
project(armbot_controller)


find_package(ament_cmake REQUIRED)


install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
