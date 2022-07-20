#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-interfaces-global-init"
#pragma ide diagnostic ignored "cert-err58-cpp"
// Declares llvm::cl::extrahelp.
#include <llvm/Support/CommandLine.h>

// Declares clang::SyntaxOnlyAction.
#include <clang/Frontend/FrontendActions.h>

#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "CallbackRegHandler.h"
#include "DataWriter.h"
#include "Matchers.h"
#include "MemberExprHandler.h"
#include "NodeDeclHandler.h"
#include "NodeNameHandler.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static llvm::cl::OptionCategory ROS2ToolCategory("ros2-tool options");

static cl::opt<std::string> OutFilename(
    "o",
    cl::desc("The name of the produced output JSON file, "
             "including the file extension.\nThe parent path must exist."),
    cl::init("result.json"), cl::cat(ROS2ToolCategory));

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
[[maybe_unused]] static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
[[maybe_unused]] static cl::extrahelp
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
  CallbackRegHandler RawCbHandler(&Writer, false);
  CallbackRegHandler CtxCbHandler(&Writer, true);
  NodeDeclHandler NodeHandler(&Writer);
  NodeNameHandler NameHandler(&Writer);
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

  addMatcher(Matchers::CallbackFuncMatcher, &CtxCbHandler);
  addMatcher(Matchers::CallbackLambdaMatcher, &CtxCbHandler);
  addMatcher(Matchers::CallbackRegistrationMatcher, &RawCbHandler);
  addMatcher(Matchers::PublisherRegistrationMatcher, &CtxCbHandler);
  addMatcher(Matchers::NodeDeclMatcher, &NodeHandler);
  addMatcher(Matchers::NodeNameMatcher, &NameHandler);

  int ExitStatus = Tool.run(newFrontendActionFactory(&Finder).get());

  Writer.write(OutFilename.c_str());
  std::cout << '[' << ExitStatus << ']' << " Done, outputs written to "
            << OutFilename << std::endl;
  return ExitStatus;
}
#pragma clang diagnostic pop