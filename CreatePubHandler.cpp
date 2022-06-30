//
// Created by max on 28.06.22.
//

#include "CreatePubHandler.h"

void schmeller::ROS2DepCheck::CreatePubHandler::run(const MatchFinder::MatchResult &Result) {

}

schmeller::ROS2DepCheck::CreatePubHandler::CreatePubHandler(DataWriter *Writer) {
    MyWriter = Writer;
}
