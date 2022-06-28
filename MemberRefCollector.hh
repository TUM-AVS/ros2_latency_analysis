//
// Created by maxim on 6/27/2022.
//

#ifndef LLVM_MEMBER_REF_COLLECTOR_HH
#define LLVM_MEMBER_REF_COLLECTOR_HH

#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <iostream>

using namespace clang;
using namespace clang::ast_matchers;

const Expr *getChild(const Expr *Parent, int ChildIndex, int ExpectedNChildren);

bool checkClass(const Expr *Expression, const Stmt::StmtClass &ExpectedClass, const std::string &NotMatchingMessage);

class MemberRefCollector : public MatchFinder::MatchCallback {
public :
    virtual void run(const MatchFinder::MatchResult &Result);

private:
    const Expr * unwrapCallbackArg(const Expr *CallbackArg);

    const Stmt * getCallbackDefinition(const DeclRefExpr *CallbackSignature);

    void getCallbackMemberRefs(Expr *Expression, std::vector<MemberExpr*> &Accumulator);
};

#endif // LLVM_MEMBER_REF_COLLECTOR_HH
