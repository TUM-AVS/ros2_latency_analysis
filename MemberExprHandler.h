//
// Created by max on 05.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_MEMBEREXPRHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_MEMBEREXPRHANDLER_H

#include <algorithm>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "ASTUtils.h"
#include "DataWriter.h"
#include "Matchers.h"
#include "ErrorPrinter.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {

class MemberExprHandler : public MatchFinder::MatchCallback {
public:
  MemberExprHandler(DataWriter *Writer, bool IncludeContext)
      : Writer(Writer), IncludeContext(IncludeContext) {}

  virtual void run(const MatchFinder::MatchResult &Result);

private:
  std::vector<MemberExpr *>
  getMemberChain(const MemberExpr *MemberRef,
                 const MatchFinder::MatchResult &Result);

  void getContextOf(const MemberExpr *MemberRef,
                    const MatchFinder::MatchResult &Result, DynTypedNode &CRef,
                    std::string &CStr);

  DataWriter *Writer;
  bool IncludeContext;
};
} // namespace schmeller::ROS2DepCheck

#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_MEMBEREXPRHANDLER_H
