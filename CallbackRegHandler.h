//
// Created by max on 18.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_CALLBACKREGHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_CALLBACKREGHANDLER_H

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

class CallbackRegHandler : public MatchFinder::MatchCallback {
public:
  CallbackRegHandler(DataWriter *Writer, bool IncludeContext) : Writer(Writer), IncludeContext(IncludeContext) {}

  virtual void run(const MatchFinder::MatchResult &Result);

  DataWriter *Writer;
  bool IncludeContext;
};

} // namespace schmeller::ROS2DepCheck

#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_CALLBACKREGHANDLER_H
