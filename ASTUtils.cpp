//
// Created by max on 28.06.22.
//

#include "ASTUtils.h"

const Expr *schmeller::ROS2DepCheck::getChild(const Expr *Parent, int ChildIndex, int ExpectedNChildren) {
    int ChildCount = 0;
    const Stmt *TheChild;
    for (const Stmt *Child: Parent->children()) {
        if (ChildCount == ChildIndex) TheChild = Child;
        ChildCount++;
    }

    if (ChildCount != ExpectedNChildren) return nullptr;
    return static_cast<const Expr *>(TheChild);
}

bool schmeller::ROS2DepCheck::checkClass(const Expr *Expression, const Stmt::StmtClass &ExpectedClass,
                                         const std::string &NotMatchingMessage) {
    if (Expression == nullptr || Expression->getStmtClass() != ExpectedClass) {
        std::cerr << NotMatchingMessage << std::endl;
        return false;
    }

    return true;
}

const Stmt *schmeller::ROS2DepCheck::getCallbackBody(const Decl *CallbackDecl) {
    return CallbackDecl->getBody();
}

void schmeller::ROS2DepCheck::getCallbackMemberRefs(Expr *Expression, std::vector<MemberExpr *> &Accumulator) {
    if (Expression->getStmtClass() == Stmt::StmtClass::MemberExprClass) {
        Accumulator.push_back(static_cast<MemberExpr *>(Expression));
    }

    for (Stmt *Child: Expression->children())
        getCallbackMemberRefs(static_cast<Expr *>(Child), Accumulator);
}

const FunctionDecl * schmeller::ROS2DepCheck::unwrapCallbackArg(const Expr *CallbackArg) {
    if (!checkClass(CallbackArg, Stmt::StmtClass::MaterializeTemporaryExprClass,
                    "Callback arg wrapper is not a temporary expression"))
        return nullptr;

    // Get Bind call
    const Expr *TheChild = getChild(CallbackArg, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::CallExprClass,
                    "Callback arg is not a bind call")) {
        if (!checkClass(TheChild, Stmt::StmtClass::LambdaExprClass,
                        "  and also not a lambda")) return nullptr;

        const CXXMethodDecl *LambdaDecl = ((const LambdaExpr *) TheChild)->getCallOperator();
        return LambdaDecl->getAsFunction();
    }

    // Get 1st arg
    const auto *TheChild2 = getChild(TheChild, 1, 4);
    if (TheChild2 == nullptr) TheChild2 = getChild(TheChild, 1, 3);
    if (!checkClass(TheChild2, Stmt::StmtClass::MaterializeTemporaryExprClass,
                    "1st arg of bind call has unexpected type")) {
        return nullptr;
    }

    TheChild = TheChild2;

    // Get &(...) expr
    TheChild = getChild(TheChild, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::UnaryOperatorClass,
                    "Bind arg is not a function reference operator"))
        return nullptr;

    TheChild = getChild(TheChild, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::DeclRefExprClass,
                    "Reference operator arg is not a decl ref"))
        return nullptr;

    return ((DeclRefExpr *) TheChild)->getDecl()->getAsFunction();
}