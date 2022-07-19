//
// Created by max on 19.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_ERRORPRINTER_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_ERRORPRINTER_H

#include <iostream>
#include <string.h>

#define __FILENAME__ (strrchr("/" __FILE__, '/') + 1)

#define _RED "\e[0;31m"
#define _YLW "\e[0;33m"
#define _RESET "\e[0m"

#define _MSG_FROM_FILE(tag, msg, color)                                        \
  std::cerr << color << '[' << tag << "][" << __FILENAME__ << "] " << msg      \
            << _RESET << std::endl

#define ERR(msg) _MSG_FROM_FILE("ERROR", msg, _RED)
#define WARN(msg) _MSG_FROM_FILE("WARN ", msg, _YLW)

#define _SRC_LOC(node, result)                                                 \
  (node->getSourceRange().printToString(*result.SourceManager))

#define SERR(node, result, msg) ERR(msg << "\n  " << _SRC_LOC(node, result))
#define SWARN(node, result, msg) WARN(msg << "\n  " << _SRC_LOC(node, result))

#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_ERRORPRINTER_H
