//
// Created by max on 30.06.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H

#include <iostream>
#include <fstream>
#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>

#include "ASTUtils.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {
    class DataWriter {
    public:
        DataWriter(std::string Filename) : OutStream(Filename) {};
        ~DataWriter() { OutStream.close(); };

        void writeNode(const CXXRecordDecl *Decl, const MatchFinder::MatchResult &Result);
        void writeFunction(const FunctionDecl *Decl, const MatchFinder::MatchResult &Result);
        void writeLambda(const LambdaExpr *Decl, const MatchFinder::MatchResult &Result);
    private:
        std::ofstream OutStream;

        std::string lvl(int Level);
    };
} // namespace schmeller::ROS2DepCheck

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_DATAWRITER_H
