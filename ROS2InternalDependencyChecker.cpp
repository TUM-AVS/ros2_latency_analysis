// Declares llvm::cl::extrahelp.
#include <llvm/Support/CommandLine.h>

// Declares clang::SyntaxOnlyAction.
#include <clang/Frontend/FrontendActions.h>

#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "CreateSubHandler.h"
#include "CreateTmrHandler.h"
#include "CreatePubHandler.h"
#include "NodeDeclHandler.h"
#include "DataWriter.h"

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
static cl::extrahelp MoreHelp(
        "\nThis tool extracts:\n"
        "* ROS 2 nodes and their member fields and methods\n"
        "* member references from ROS 2 subscription and timer callbacks\n"
        "* metadata (source ranges, qualified names, value types, etc.)"
);

int main(int argc, const char **argv) {
    auto ExpectedParser = CommonOptionsParser::create(argc, argv, ROS2ToolCategory);
    if (!ExpectedParser) {
        // Fail gracefully for unsupported options.
        llvm::errs() << ExpectedParser.takeError();
        return 1;
    }
    CommonOptionsParser &OptionsParser = ExpectedParser.get();
    ClangTool Tool(OptionsParser.getCompilations(),
                   OptionsParser.getSourcePathList());

    StatementMatcher CreateSubMatcher = callExpr(callee(functionDecl(hasName("create_subscription"))))
            .bind("create_sub_call");
    StatementMatcher CreateTmrMatcher = callExpr(callee(functionDecl(hasName("create_timer"))))
            .bind("create_tmr_call");
    StatementMatcher CreatePubMatcher = callExpr(callee(functionDecl(hasName("create_publisher"))))
            .bind("create_pub_call");
    DeclarationMatcher NodeDeclMatcher = cxxRecordDecl(hasAnyBase(hasType(cxxRecordDecl(hasName("rclcpp::Node")))))
            .bind("node_decl");

    DataWriter Writer("node.yml");

    CreateSubHandler SubHandler(&Writer);
    CreateTmrHandler TmrHandler(&Writer);
    CreatePubHandler PubHandler(&Writer);
    NodeDeclHandler NodeHandler(&Writer);
    MatchFinder Finder;
    Finder.addMatcher(CreateSubMatcher, &SubHandler);
    Finder.addMatcher(CreateTmrMatcher, &TmrHandler);
    Finder.addMatcher(CreatePubMatcher, &PubHandler);
    Finder.addMatcher(NodeDeclMatcher, &NodeHandler);

    auto ExitStatus = Tool.run(newFrontendActionFactory(&Finder).get());

    // We get a segfault on exit, so make sure the reader is closed at least...
    // (Yes, this hurts as much to write as it does to read :c )
    Writer.~DataWriter();
    std::cout << '[' << ExitStatus << ']' << " Done, outputs written." << std::endl;
    return ExitStatus;
}