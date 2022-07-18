//
// Created by max on 08.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_COMPOUNDHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_COMPOUNDHANDLER_H

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <iostream>
#include <string>
#include <vector>

#include "ASTUtils.h"
#include "DataWriter.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {

    class CompoundHandler : public MatchFinder::MatchCallback {
    public:
        CompoundHandler(const std::vector<MatchFinder::MatchCallback *> &Handlers) : Handlers(Handlers) {}

        virtual void run(const MatchFinder::MatchResult &Result);

    private:
        std::vector<MatchFinder::MatchCallback *> Handlers;
    };

} // namespace schmeller::ROS2DepCheck

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_COMPOUNDHANDLER_H
