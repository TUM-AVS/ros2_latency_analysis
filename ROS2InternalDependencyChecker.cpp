// Declares llvm::cl::extrahelp.
#include <llvm/Support/CommandLine.h>

// Declares clang::SyntaxOnlyAction.
#include <clang/Frontend/FrontendActions.h>

#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "CompoundHandler.h"
#include "DataWriter.h"
#include "Matchers.h"
#include "MemberExprHandler.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static llvm::cl::OptionCategory ROS2ToolCategory("ros2-tool options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp
    MoreHelp("\nThis tool extracts:\n"
             "* ROS 2 nodes and their member fields and methods\n"
             "* member references from ROS 2 subscription and timer callbacks\n"
             "* metadata (source ranges, qualified names, value types, etc.)");

int main(int argc, const char **argv) {
  auto ExpectedParser =
      CommonOptionsParser::create(argc, argv, ROS2ToolCategory);
  if (!ExpectedParser) {
    // Fail gracefully for unsupported options.
    llvm::errs() << ExpectedParser.takeError();
    return 1;
  }
  CommonOptionsParser &OptionsParser = ExpectedParser.get();
  ClangTool Tool(OptionsParser.getCompilations(),
                 OptionsParser.getSourcePathList());

  DataWriter Writer;

  MemberExprHandler RawMemberHandler(&Writer, false);
  MemberExprHandler CtxMemberHandler(&Writer, true);
  MatchFinder Finder;

  auto addMatcher = [&Finder](auto Matcher, auto Handler) {
    Finder.addMatcher(traverse(clang::TK_IgnoreUnlessSpelledInSource, Matcher),
                      Handler);
  };

  addMatcher(Matchers::ChainedMemberMatcher, &RawMemberHandler);
  addMatcher(Matchers::MemberIsLHSMatcher, &CtxMemberHandler);
  addMatcher(Matchers::MemberIsRHSMatcher, &CtxMemberHandler);
  addMatcher(Matchers::MemberInDeclMatcher, &CtxMemberHandler);
  addMatcher(Matchers::MemberIsCalleeMatcher, &CtxMemberHandler);
  addMatcher(Matchers::MemberIsCallArgMatcher, &CtxMemberHandler);
  addMatcher(Matchers::AnyPublishCallMatcher, &CtxMemberHandler);
  addMatcher(Matchers::MemberPublishCallMatcher, &CtxMemberHandler);

  // addMatcher(Matchers::FuncContainingPubCallMatcher, &MemberHandler);
  // addMatcher(Matchers::CallbackFuncMatcher, &MemberHandler);
  // addMatcher(Matchers::CallbackLambdaMatcher, &MemberHandler);
  // addMatcher(Matchers::CallbackRegistrationMatcher, &MemberHandler);

  int ExitStatus = Tool.run(newFrontendActionFactory(&Finder).get());

  const char *SourceFilename = OptionsParser.getSourcePathList()[0].c_str();
  char *SourceFilenameMutable = (char *)malloc(strlen(SourceFilename) + 1);
  strcpy(SourceFilenameMutable, SourceFilename);

  for (int i = 0; i < strlen(SourceFilenameMutable) + 1; ++i) {
    if (SourceFilenameMutable[i] == '/')
      SourceFilenameMutable[i] = '-';
  }

  Writer.write(SourceFilenameMutable);
  std::cout << '[' << ExitStatus << ']' << " Done, outputs written."
            << std::endl;
  return ExitStatus;
}