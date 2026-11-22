cmake_minimum_required(VERSION 3.10)
project(line_maze_solver)

find_package(ament_cmake REQUIRED)

install(DIRECTORY ai_models config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/nodes/cnn_follower.py
  src/nodes/pid_follower.py
  src/nodes/frame_viewer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
