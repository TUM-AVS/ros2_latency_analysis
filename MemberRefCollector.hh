//
// Created by maxim on 6/27/2022.
//

#ifndef LLVM_MEMBER_REF_COLLECTOR_HH
#define LLVM_MEMBER_REF_COLLECTOR_HH

#include "clang/ASTMatchers/ASTMatchers.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"

using namespace clang;
using namespace clang::ast_matchers;

class MemberRefCollector : public MatchFinder::MatchCallback {
public :
  virtual void run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr *expr = Result.Nodes.getNodeAs<clang::CallExpr>("create_sub_call")) {
      const Expr *callbackArg = expr->getArg(2);
      callbackArg->dump();
    }
  }
};

#endif // LLVM_MEMBER_REF_COLLECTOR_HH
