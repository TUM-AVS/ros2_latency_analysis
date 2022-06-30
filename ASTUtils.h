//
// Created by max on 28.06.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_ASTUTILS_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_ASTUTILS_H

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <iostream>

using namespace clang;
using namespace clang::ast_matchers;

namespace schmeller::ROS2DepCheck {
    const Expr *getChild(const Expr *Parent, int ChildIndex, int ExpectedNChildren);

    bool checkClass(const Expr *Expression, const Stmt::StmtClass &ExpectedClass, const std::string &NotMatchingMessage);

    const Stmt * getCallbackBody(const Decl *CallbackDecl);

    void getCallbackMemberRefs(Expr *Expression, std::vector<MemberExpr*> &Accumulator);

    const FunctionDecl * unwrapCallbackArg(const Expr *CallbackArg);

} // namespace schmeller::ROS2DepCheck

#endif //ROS2_INTERNAL_DEPENDENCY_CHECKER_ASTUTILS_H
