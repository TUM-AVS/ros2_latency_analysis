//
// Created by max on 18.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H

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

namespace schmeller {
namespace ROS2DepCheck {

class NodeDeclHandler : public MatchFinder::MatchCallback {
public:
  NodeDeclHandler(DataWriter *Writer) : Writer(Writer) {}

  virtual void run(const MatchFinder::MatchResult &Result);

  DataWriter *Writer;
};

} // namespace ROS2DepCheck
} // namespace schmeller

#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_NODEDECLHANDLER_H
