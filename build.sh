clang_libs="-lclangAnalysis -lclangAnalysisFlowSensitive -lclangAPINotes -lclangApplyReplacements -lclangARCMigrate -lclangAST -lclangASTMatchers -lclangBasic -lclangChangeNamespace -lclangCodeGen -lclang-cpp14 -lclang-cpp -lclang-cpp -lclangCrossTU -lclangDaemon -lclangDaemonTweaks -lclangDependencyScanning -lclangDirectoryWatcher -lclangdMonitoringServiceProto -lclangDoc -lclangdRemoteIndex -lclangdRemoteIndexProto -lclangdRemoteIndexServiceProto -lclangdRemoteMarshalling -lclangDriver -lclangdSupport -lclangDynamicASTMatchers -lclangEdit -lclangFormat -lclangFrontend -lclangFrontendTool -lclangHandleCXX -lclangHandleLLVM -lclangIncludeFixer -lclangIncludeFixerPlugin -lclangIndex -lclangIndexSerialization -lclangInterpreter -lclangLex -lclangMove -lclangParse -lclangQuery -lclangReorderFields -lclangRewrite -lclangRewriteFrontend -lclangSema -lclangSerialization -lclang -lclang -lclangStaticAnalyzerCheckers -lclangStaticAnalyzerCore -lclangStaticAnalyzerFrontend -lclangTesting -lclangTidy -lclangTidyAbseilModule -lclangTidyAlteraModule -lclangTidyAndroidModule -lclangTidyBoostModule -lclangTidyBugproneModule -lclangTidyCERTModule -lclangTidyConcurrencyModule -lclangTidyCppCoreGuidelinesModule -lclangTidyDarwinModule -lclangTidyFuchsiaModule -lclangTidyGoogleModule -lclangTidyHICPPModule -lclangTidyLinuxKernelModule -lclangTidyLLVMLibcModule -lclangTidyLLVMModule -lclangTidyMain -lclangTidyMiscModule -lclangTidyModernizeModule -lclangTidyMPIModule -lclangTidyObjCModule -lclangTidyOpenMPModule -lclangTidyPerformanceModule -lclangTidyPlugin -lclangTidyPortabilityModule -lclangTidyReadabilityModule -lclangTidyUtils -lclangTidyZirconModule -lclangTooling -lclangToolingASTDiff -lclangToolingCore -lclangToolingInclusions -lclangToolingRefactoring -lclangToolingSyntax -lclangTransformer"

clang++ --std=c++17 -Wno-c++17-extensions -g3 -c $(llvm-config --cxxflags) *.cpp
clang++ *.o $(llvm-config --ldflags --libs) $clang_libs -lpthread -lncurses -o dep-check