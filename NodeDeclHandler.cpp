//
// Created by max on 18.07.22.
//

#include "NodeDeclHandler.h"

namespace schmeller::ROS2DepCheck {
void NodeDeclHandler::run(const MatchFinder::MatchResult &Result) {
  const CXXRecordDecl *NodeDecl =
      Result.Nodes.getNodeAs<CXXRecordDecl>("node_decl");

  if (NodeDecl == NULL) {
    ERR("Node decl not found in node decl handler");
    return;
  }

  Writer->addAtPath("nodes", Writer->nodeToJson(NodeDecl, Result));
}
} // namespace schmeller::ROS2DepCheck
