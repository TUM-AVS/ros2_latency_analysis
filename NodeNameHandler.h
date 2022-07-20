//
// Created by max on 20.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_NODENAMEHANDLER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_NODENAMEHANDLER_H

#include "ASTUtils.h"
#include "DataWriter.h"
#include "ErrorPrinter.h"
#include "Matchers.h"

using namespace clang;
using namespace clang::ast_matchers;
using namespace schmeller::ROS2DepCheck;

namespace schmeller::ROS2DepCheck {

class NodeNameHandler : public MatchFinder::MatchCallback {
public:
  NodeNameHandler(DataWriter *Writer) : Writer(Writer) {}

  virtual void run(const MatchFinder::MatchResult &Result);

  DataWriter *Writer;
};
} // namespace schmeller::ROS2DepCheck

#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_NODENAMEHANDLER_H
