set(LLVM_LINK_COMPONENTS support)

add_clang_executable(ros2-internal-dependency-checker
  ROS2InternalDependencyChecker.cpp
  MemberRefCollector.cpp
  )
target_link_libraries(ros2-internal-dependency-checker
  PRIVATE
  clangAST
  clangASTMatchers
  clangBasic
  clangFrontend
  clangSerialization
  clangTooling
  )