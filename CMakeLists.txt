cmake_minimum_required(VERSION 3.22)

project(ros2-internal-dependency-checker)

include_directories("/usr/lib/llvm-13/include")

add_executable(ros2-internal-dependency-checker
  ROS2InternalDependencyChecker.cpp
        ASTUtils.cpp ASTUtils.h DataWriter.cpp DataWriter.h MemberExprHandler.h Matchers.h MemberExprHandler.cpp Matchers.cpp NodeDeclHandler.cpp NodeDeclHandler.h CallbackRegHandler.cpp CallbackRegHandler.h ErrorPrinter.h)

set_property(TARGET ros2-internal-dependency-checker PROPERTY CXX_STANDARD 20)
target_compile_options(ros2-internal-dependency-checker PRIVATE -w)
target_link_directories(ros2-internal-dependency-checker PRIVATE /usr/lib/llvm-14/lib)

find_package(nlohmann_json 3.2.0 REQUIRED)

target_link_libraries(ros2-internal-dependency-checker PRIVATE
  nlohmann_json::nlohmann_json 
  clangAnalysis
  clangAnalysisFlowSensitive
  clangAPINotes
  clangApplyReplacements
  clangARCMigrate
  clangAST
  clangASTMatchers
  clangBasic
  clangChangeNamespace
  clangCodeGen
  clang-cpp14
  clang-cpp
  clang-cpp
  clangCrossTU
  clangDaemon
  clangDaemonTweaks
  clangDependencyScanning
  clangDirectoryWatcher
  clangdMonitoringServiceProto
  clangDoc
  clangdRemoteIndex
  clangdRemoteIndexProto
  clangdRemoteIndexServiceProto
  clangdRemoteMarshalling
  clangDriver
  clangdSupport
  clangDynamicASTMatchers
  clangEdit
  clangFormat
  clangFrontend
  clangFrontendTool
  clangHandleCXX
  clangHandleLLVM
  clangIncludeFixer
  clangIncludeFixerPlugin
  clangIndex
  clangIndexSerialization
  clangInterpreter
  clangLex
  clangMove
  clangParse
  clangQuery
  clangReorderFields
  clangRewrite
  clangRewriteFrontend
  clangSema
  clangSerialization
  clang
  clang
  clangStaticAnalyzerCheckers
  clangStaticAnalyzerCore
  clangStaticAnalyzerFrontend
  clangTesting
  clangTidy
  clangTidyAbseilModule
  clangTidyAlteraModule
  clangTidyAndroidModule
  clangTidyBoostModule
  clangTidyBugproneModule
  clangTidyCERTModule
  clangTidyConcurrencyModule
  clangTidyCppCoreGuidelinesModule
  clangTidyDarwinModule
  clangTidyFuchsiaModule
  clangTidyGoogleModule
  clangTidyHICPPModule
  clangTidyLinuxKernelModule
  clangTidyLLVMLibcModule
  clangTidyLLVMModule
  clangTidyMain
  clangTidyMiscModule
  clangTidyModernizeModule
  clangTidyMPIModule
  clangTidyObjCModule
  clangTidyOpenMPModule
  clangTidyPerformanceModule
  clangTidyPlugin
  clangTidyPortabilityModule
  clangTidyReadabilityModule
  clangTidyUtils
  clangTidyZirconModule
  clangTooling
  clangToolingASTDiff
  clangToolingCore
  clangToolingInclusions
  clangToolingRefactoring
  clangToolingSyntax
  clangTransformer
  )