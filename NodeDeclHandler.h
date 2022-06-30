//
// Created by max on 30.06.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <iostream>

#include "ASTUtils.h"
#include "DataWriter.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {

    class NodeDeclHandler : public MatchFinder::MatchCallback {
    public :
        NodeDeclHandler(DataWriter *Writer);

        virtual void run(const MatchFinder::MatchResult &Result);

    private:
        DataWriter *MyWriter;
    };

} // namespace schmeller::ROS2DepCheck

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H
