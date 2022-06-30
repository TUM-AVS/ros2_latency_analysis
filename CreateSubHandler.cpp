//
// Created by max on 28.06.22.
//

#include "CreateSubHandler.h"

void schmeller::ROS2DepCheck::CreateSubHandler::run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr *CreateSubCall = Result.Nodes.getNodeAs<clang::CallExpr>("create_sub_call")) {
        if (CreateSubCall->getNumArgs() < 3) {
            std::cerr << "Expression has " << CreateSubCall->getNumArgs() << " args, expected 3 or more. Ignoring."
                      << std::endl;
            return;
        }
        const Expr *CallbackArg = CreateSubCall->getArg(2);
        const FunctionDecl *CallbackDecl = unwrapCallbackArg(CallbackArg);
        if (CallbackDecl == nullptr) return;

        MyWriter->writeFunction(CallbackDecl, Result);
    }
}

schmeller::ROS2DepCheck::CreateSubHandler::CreateSubHandler(DataWriter *Writer) {
    MyWriter = Writer;
}
