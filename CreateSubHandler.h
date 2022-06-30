//
// Created by max on 28.06.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_CREATESUBHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_CREATESUBHANDLER_H

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <iostream>

#include "ASTUtils.h"
#include "DataWriter.h"

using namespace clang;
using namespace clang::ast_matchers;

using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {
    class CreateSubHandler : public MatchFinder::MatchCallback {
    public :
        CreateSubHandler(DataWriter *Writer);

        virtual void run(const MatchFinder::MatchResult &Result);

    private:
        DataWriter *MyWriter;
    };
}

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_CREATESUBHANDLER_H
