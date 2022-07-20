//
// Created by max on 20.07.22.
//

#include "NodeNameHandler.h"

namespace schmeller::ROS2DepCheck {
void NodeNameHandler::run(const MatchFinder::MatchResult &Result) {
  const auto *NodeDecl = Result.Nodes.getNodeAs<CXXRecordDecl>("node_decl");

  const auto *CtorInit =
      Result.Nodes.getNodeAs<CXXCtorInitializer>("ctor_initializer");

  const auto *NodeName = Result.Nodes.getNodeAs<StringLiteral>("node_name");
  const auto *NodeNamespace =
      Result.Nodes.getNodeAs<StringLiteral>("node_namespace");

  if (NodeDecl == nullptr) {
    ERR("Node decl not found in node decl handler");
    return;
  }

  if (CtorInit == nullptr) {
    ERR("Constructor init not found but bound");
    return;
  }

  if (NodeName == nullptr) {
    SERR(CtorInit, Result, "CtorInitializer contains no literal node name");
    return;
  }

  NodeDecl = NodeDecl->getCanonicalDecl();

  json J = Writer->nodeToJson(NodeDecl, Result);
  J["ros_name"] = NodeName->getString().str();

  if (NodeNamespace != nullptr)
    J["ros_namespace"] = NodeNamespace->getString().str();

  Writer->addAtPath("nodes", J);
}
} // namespace schmeller::ROS2DepCheck