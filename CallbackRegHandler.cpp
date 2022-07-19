//
// Created by max on 18.07.22.
//

#include "CallbackRegHandler.h"

namespace schmeller::ROS2DepCheck {
void CallbackRegHandler::run(const MatchFinder::MatchResult &Result) {

  /*********************************************
   * Collect potentially bound AST nodes
   *********************************************/

  const CallExpr *Call = Result.Nodes.getNodeAs<CallExpr>("call");
  const MemberExpr *MemberRef =
      Result.Nodes.getNodeAs<MemberExpr>(Matchers::MemberReference);
  const StringLiteral *Topic =
      Result.Nodes.getNodeAs<StringLiteral>("topic_name");
  const bool HasCbDecl = Result.Nodes.getMap().count("decl") > 0;

  if (Call == NULL) {
    ERR("Call expression not found in callback registration handler");
    return;
  }

  json OutJson = {};

  /*********************************************
   * Check type of created object and verify
   *********************************************/

  const std::string CreateCallName = Call->getDirectCallee()->getNameAsString();
  const char *JsonPath;

  if (CreateCallName == "create_publisher") {
    OutJson["type"] = "publisher";
    JsonPath = "publishers";
  } else if (CreateCallName == "create_subscription") {
    OutJson["type"] = "subscription";
    JsonPath = "subscriptions";
  } else if (CreateCallName == "create_timer") {
    OutJson["type"] = "timer";
    JsonPath = "timers";
  } else {
    ERR("Callback creation using unknown method '"
        << CreateCallName << "'\n"
        << Call->getSourceRange().printToString(*Result.SourceManager));
    return;
  }

  if (IncludeContext &&
      (JsonPath == "subscriptions" || JsonPath == "publishers") &&
      Topic == NULL) {
    ERR("Call to " << CreateCallName << " has no topic\n"
                   << Call->getSourceRange().printToString(
                          *Result.SourceManager));
    return;
  }

  if (IncludeContext && JsonPath == "publishers" && MemberRef == NULL) {
    ERR("Created publisher is not "
        "assigned to a node member\n"
        << Call->getSourceRange().printToString(*Result.SourceManager));
    return;
  }

  if (IncludeContext && (JsonPath == "subscriptions" || JsonPath == "timers") &&
      !HasCbDecl) {
    ERR("No callback reference found in call to "
        << CreateCallName << "\n"
        << Call->getSourceRange().printToString(*Result.SourceManager));
    return;
  }

  /*********************************************
   * Get topic name if present
   *********************************************/

  if (Topic != NULL) {
    std::string TopicName = Topic->getString().str();
    OutJson["topic"] = TopicName;
  }

  /*********************************************
   * Get callback function decl if present
   *********************************************/

  if (HasCbDecl) {
    /*********************************************
     * Try getting CB as functionDecl
     *********************************************/

    const FunctionDecl *CbFuncDecl =
        Result.Nodes.getNodeAs<FunctionDecl>("decl");

    /*********************************************
     * Else, try getting CB as lambdaExpr
     *********************************************/

    if (CbFuncDecl == NULL) {
      const LambdaExpr *CbLambda = Result.Nodes.getNodeAs<LambdaExpr>("decl");
      if (CbLambda == NULL) {
        ERR("Callback declaration not found but bound\n"
            << Call->getSourceRange().printToString(*Result.SourceManager));
        return;
      }

      CbFuncDecl = CbLambda->getCallOperator();
    }

    /*********************************************
     * Write found callback decl to JSON
     *********************************************/

    OutJson["callback"] = Writer->functionDeclToJson(CbFuncDecl, Result);
  }

  /*********************************************
   * Write member reference if found
   *********************************************/

  if (MemberRef != NULL) {
    OutJson["member"] = Writer->memberToJson(MemberRef, Result);
  }

  Writer->addAtPath(JsonPath, OutJson);
}
} // namespace schmeller::ROS2DepCheck
