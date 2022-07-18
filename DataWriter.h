//
// Created by max on 30.06.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>
#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "ASTUtils.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace nlohmann;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {
    class DataWriter {
    public:
        json nodeToJson(const CXXRecordDecl *Decl, const MatchFinder::MatchResult &Result);
        json functionDeclToJson(const FunctionDecl *Decl, const MatchFinder::MatchResult &Result);
        json memberChainToJson(const std::vector<MemberExpr *> &MemberChain, const MatchFinder::MatchResult &Result);
        json memberToJson(const MemberExpr *Member, const MatchFinder::MatchResult &Result);
        json sourceRangeToJson(const SourceRange &Range, const MatchFinder::MatchResult &Result);

        void addAtPath(const char *Path, const json &Node);

        void write(const char *Filename);
    private:
        json Json;
    };
} // namespace schmeller::ROS2DepCheck

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H
