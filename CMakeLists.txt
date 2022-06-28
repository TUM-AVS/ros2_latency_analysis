cmake_minimum_required(VERSION 3.22)

project(ros2-internal-dependency-checker)

include_directories("/usr/lib/llvm-14/include")

add_executable(ros2-internal-dependency-checker
  ROS2InternalDependencyChecker.cpp
  MemberRefCollector.cpp
  )

target_link_directories(ros2-internal-dependency-checker PRIVATE /usr/lib/llvm-14/lib)

target_link_libraries(ros2-internal-dependency-checker
  PRIVATE
  clangAST
  clangASTMatchers
  clangBasic
  clangFrontend
  clangSerialization
  clangTooling
  )