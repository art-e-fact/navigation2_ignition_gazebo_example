cmake_minimum_required(VERSION 3.5)
project(sam_bot_nav2_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY  launch rviz config world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
