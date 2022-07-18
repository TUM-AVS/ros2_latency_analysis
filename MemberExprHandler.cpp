//
// Created by max on 05.07.22.
//

#include "MemberExprHandler.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

void schmeller::ROS2DepCheck::MemberExprHandler::run(
    const MatchFinder::MatchResult &Result) {
  /*********************************************
   * Validate the matched MemberExpr
   *********************************************/

  const MemberExpr *MemberRef =
      Result.Nodes.getNodeAs<MemberExpr>(Matchers::MemberReference);
  if (!MemberRef) {
    std::cerr << "[ERROR][MemberExprHandler] Match on MemberExpr has no "
                 "MemberExpr bound to 'ref'"
              << std::endl;
    return;
  }

  /*********************************************
   * Find and validate context statement
   *********************************************/

  DynTypedNode CRef = DynTypedNode::create(*MemberRef);
  std::string CStr("none");

  if (IncludeContext)
    getContextOf(MemberRef, Result, CRef, CStr);

  /*********************************************
   * Output member info and statement context
   *********************************************/

  json J = {{"context",
             {{"statement",
               {{"source_range",
                 Writer->sourceRangeToJson(CRef.getSourceRange(), Result)}}},
              {"access_type", CStr}}},
            {"member", Writer->memberChainToJson(
                           getMemberChain(MemberRef, Result), Result)}};

  /*********************************************
   * Find and validate containing method
   *********************************************/

  const CXXMethodDecl *ContainingMethod =
      Result.Nodes.getNodeAs<CXXMethodDecl>("containing_method");
  if (!ContainingMethod) {
    std::cerr
        << "[WARN ][MemberExprHandler] MemberExpr not inside a CXXMethod.\n  "
        << MemberRef->getSourceRange().printToString(*Result.SourceManager)
        << std::endl;
  }

  /*********************************************
   * Output method context
   *********************************************/

  if (ContainingMethod) {
    J["context"]["method"] = {
        {"id", ContainingMethod->getID()},
        {"qualified_name", ContainingMethod->getQualifiedNameAsString()},
        {"source_range", Writer->sourceRangeToJson(
                             ContainingMethod->getSourceRange(), Result)}};
  }

  /*********************************************
   * Find and process containing ROS node
   *********************************************/

  const CXXRecordDecl *ContainingNode =
      Result.Nodes.getNodeAs<CXXRecordDecl>("node_decl");
  if (!ContainingNode) {
    std::cerr << "[ERROR][MemberExprHandler] MemberExpr has no declaration "
                 "in ROS Node.\n  "
              << MemberRef->getSourceRange().printToString(
                     *Result.SourceManager)
              << std::endl;
    return;
  }

  /*********************************************
   * Output node context
   *********************************************/

  J["context"]["node"] = {
      {"id", ContainingNode->getID()},
      {"qualified_name", ContainingNode->getQualifiedNameAsString()},
      {"source_range",
       Writer->sourceRangeToJson(ContainingNode->getSourceRange(), Result)}};

  /*********************************************
   * Write JSON
   *********************************************/

  Writer->addAtPath(CStr.c_str(), J);
}

std::vector<MemberExpr *>
schmeller::ROS2DepCheck::MemberExprHandler::getMemberChain(
    const MemberExpr *MemberRef, const MatchFinder::MatchResult &Result) {
  std::vector<MemberExpr *> MemberChain;

  const Stmt *CurRef = MemberRef;
  while (CurRef) {
    if (CurRef->getStmtClass() == Stmt::StmtClass::CXXOperatorCallExprClass) {
      CurRef = (Stmt *)((CXXOperatorCallExpr *)CurRef)->getCallee();
      continue;
    }

    if (CurRef->getStmtClass() == Stmt::StmtClass::MemberExprClass)
      MemberChain.push_back((MemberExpr *)CurRef);

    int I = 0;
    for (const auto *Child : CurRef->children()) {
      if (I++ != 0)
        std::cerr << "[ERROR][MemberExprHandler] More than one child in "
                     "MemberRef chain\n  "
                  << Child->getSourceRange().printToString(
                         *Result.SourceManager)
                  << std::endl;
      CurRef = Child;
    }

    if (I == 0)
      break;
  }

  return MemberChain;
}

void schmeller::ROS2DepCheck::MemberExprHandler::getContextOf(
    const MemberExpr *MemberRef, const MatchFinder::MatchResult &Result,
    DynTypedNode &CRef, std::string &CStr) {
  std::vector<const char *> CNames = {"write", "read", "read",
                                      "call",  "arg",  "pub"};

  std::vector<const char *> CBinds = {
      Matchers::OpWithLhsMember,     Matchers::OpWithRhsMember,
      Matchers::DeclUsingMember,     Matchers::CallToMember,
      Matchers::CallWithMemberAsArg, Matchers::PublishOfMemberPublisher};

  std::vector<const DynTypedNode *> CNodes;
  std::transform(CBinds.begin(), CBinds.end(), std::back_inserter(CNodes),
                 [&Result](const char *Bind) {
                   if (Result.Nodes.getMap().count(Bind)) {
                     if (std::string(Bind) ==
                         std::string(Matchers::DeclUsingMember))
                       std::cout << "DeclUsingMember" << std::endl;
                     return &Result.Nodes.getMap().at(Bind);
                   }
                   return (const DynTypedNode *)NULL;
                 });

  std::vector<bool> CActive;
  std::transform(CNodes.begin(), CNodes.end(), std::back_inserter(CActive),
                 [](const DynTypedNode *Ref) { return Ref != NULL; });

  int NActive = std::count(CActive.begin(), CActive.end(), true);
  if (NActive > 1) {
    std::cerr << "[ERROR][MemberExprHandler] Multiple contexts for the "
                 "member reference were detected at once:"
              << CActive[0] << CActive[1] << CActive[2] << CActive[3]
              << CActive[4] << CActive[5] << "\n  "
              << MemberRef->getSourceRange().printToString(
                     *Result.SourceManager)
              << std::endl;
    return;
  }

  if (NActive == 0) {
    std::cerr << "[WARN ][MemberExprHandler] No context detected for the "
                 "member reference.\n  "
              << MemberRef->getSourceRange().printToString(
                     *Result.SourceManager)
              << std::endl;
  }

  int CIndex =
      std::find(CActive.begin(), CActive.end(), true) - CActive.begin();

  if (CIndex < CActive.size()) {
    CRef = *CNodes[CIndex];
    CStr = std::string(CNames[CIndex]);
  }
}
