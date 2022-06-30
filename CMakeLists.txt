cmake_minimum_required(VERSION 3.22)

project(ros2-internal-dependency-checker)

include_directories("/usr/lib/llvm-14/include")

add_executable(ros2-internal-dependency-checker
  ROS2InternalDependencyChecker.cpp
        CreateSubHandler.cpp CreateSubHandler.h CreateTmrHandler.cpp CreateTmrHandler.h CreatePubHandler.cpp CreatePubHandler.h ASTUtils.cpp ASTUtils.h NodeDeclHandler.cpp NodeDeclHandler.h DataWriter.cpp DataWriter.h)

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