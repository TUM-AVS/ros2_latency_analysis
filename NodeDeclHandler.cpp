//
// Created by max on 30.06.22.
//

#include "NodeDeclHandler.h"

void schmeller::ROS2DepCheck::NodeDeclHandler::run(const MatchFinder::MatchResult &Result) {
    if (const CXXRecordDecl *NodeDecl = Result.Nodes.getNodeAs<clang::CXXRecordDecl>("node_decl")) {
        MyWriter->writeNode(NodeDecl, Result);
    }
}

schmeller::ROS2DepCheck::NodeDeclHandler::NodeDeclHandler(DataWriter *Writer) {
    MyWriter = Writer;
}
