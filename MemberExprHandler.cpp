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
    ERR("Match on MemberExpr has no MemberExpr bound to 'ref'");
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
    SWARN(MemberRef, Result, "MemberExpr not inside a CXXMethod.");
  }

  /*********************************************
   * Output method context
   *********************************************/

  if (ContainingMethod) {
    ContainingMethod = ContainingMethod->getCanonicalDecl();
    J["context"]["method"] = {
        {"id", ContainingMethod->getID()},
        {"qualified_name", ContainingMethod->getQualifiedNameAsString()},
        {"source_range", Writer->sourceRangeToJson(
                             ContainingMethod->getSourceRange(), Result)},
        {"is_lambda", false}};
  }

  /*********************************************
   * Find and validate containing lambda
   *********************************************/

  const LambdaExpr *ContainingLambda =
      Result.Nodes.getNodeAs<LambdaExpr>("containing_lambda");

  /*********************************************
   * Output lambda context
   *********************************************/

  if (ContainingLambda) {
    ContainingMethod = ContainingLambda->getCallOperator()->getCanonicalDecl();
    J["context"]["method"] = {
        {"id", ContainingMethod->getID()},
        {"qualified_name", ContainingMethod->getQualifiedNameAsString()},
        {"source_range", Writer->sourceRangeToJson(
                             ContainingMethod->getSourceRange(), Result)},
        {"is_lambda", true}};
  }

  /*********************************************
   * Find and process containing ROS node
   *********************************************/

  const CXXRecordDecl *ContainingNode =
      Result.Nodes.getNodeAs<CXXRecordDecl>("node_decl");
  if (!ContainingNode) {
    SERR(MemberRef, Result, "MemberExpr has no declaration in ROS Node.");
    return;
  }

  /*********************************************
   * Output node context
   *********************************************/

  J["context"]["node"] = {
      {"id", ContainingNode->getCanonicalDecl()->getID()},
      {"qualified_name", ContainingNode->getQualifiedNameAsString()},
      {"source_range",
       Writer->sourceRangeToJson(ContainingNode->getSourceRange(), Result)}};

  /*********************************************
   * Write JSON
   *********************************************/

  std::string JsonPath = (std::stringstream() << "accesses/" << CStr).str();
  Writer->addAtPath(JsonPath.c_str(), J);
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
        SERR(Child, Result, "More than one child in MemberRef chain");
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
                     return &Result.Nodes.getMap().at(Bind);
                   }
                   return (const DynTypedNode *)nullptr;
                 });

  std::vector<bool> CActive;
  std::transform(CNodes.begin(), CNodes.end(), std::back_inserter(CActive),
                 [](const DynTypedNode *Ref) { return Ref != nullptr; });

  int NActive = std::count(CActive.begin(), CActive.end(), true);
  if (NActive > 1) {
    SERR(MemberRef, Result,
         "Multiple contexts for the member reference were detected at once:"
             << CActive[0] << CActive[1] << CActive[2] << CActive[3]
             << CActive[4] << CActive[5]);
    return;
  }

  if (NActive == 0) {
    SWARN(MemberRef, Result, "No context detected for the member reference.");
  }

  int CIndex =
      std::find(CActive.begin(), CActive.end(), true) - CActive.begin();

  if (CIndex < CActive.size()) {
    CRef = *CNodes[CIndex];
    CStr = std::string(CNames[CIndex]);
  }
}
