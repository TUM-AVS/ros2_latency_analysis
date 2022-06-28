// Declares llvm::cl::extrahelp.
#include <llvm/Support/CommandLine.h>

// Declares clang::SyntaxOnlyAction.
#include <clang/Frontend/FrontendActions.h>

#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "MemberRefCollector.hh"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static llvm::cl::OptionCategory ROS2ToolCategory("ros2-tool options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp MoreHelp("\nMore help text...\n");

int main(int argc, const char **argv) {
  auto ExpectedParser = CommonOptionsParser::create(argc, argv, ROS2ToolCategory);
  if (!ExpectedParser) {
    // Fail gracefully for unsupported options.
    llvm::errs() << ExpectedParser.takeError();
    return 1;
  }
  CommonOptionsParser& OptionsParser = ExpectedParser.get();
  ClangTool Tool(OptionsParser.getCompilations(),
                 OptionsParser.getSourcePathList());

  StatementMatcher CreateSubMatcher = callExpr(callee(functionDecl(hasName("create_subscription")))).bind("create_sub_call");
  StatementMatcher CreateTmrMatcher = callExpr(callee(functionDecl(hasName("create_timer")))).bind("create_tmr_call");
  StatementMatcher CreatePubMatcher = callExpr(callee(functionDecl(hasName("create_publisher")))).bind("create_pub_call");

  MemberRefCollector Collector;
  MatchFinder Finder;
  Finder.addMatcher(CreateSubMatcher, &Collector);

  return Tool.run(newFrontendActionFactory(&Finder).get());
}