//
// Created by max on 08.07.22.
//

#include "CompoundHandler.h"

void schmeller::ROS2DepCheck::CompoundHandler::run(const MatchFinder::MatchResult &Result) {
    for (auto *Handler: Handlers) {
        Handler->run(Result);
    }
}