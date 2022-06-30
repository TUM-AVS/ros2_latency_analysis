//
// Created by max on 28.06.22.
//

#include "CreateTmrHandler.h"

void schmeller::ROS2DepCheck::CreateTmrHandler::run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr *CreateTmrCall = Result.Nodes.getNodeAs<clang::CallExpr>("create_tmr_call")) {
        if (CreateTmrCall->getNumArgs() < 4) {
            std::cerr << "Expression has " << CreateTmrCall->getNumArgs() << " args, expected 4 or more. Ignoring."
                      << std::endl;
            return;
        }
        const Expr *CallbackArg = CreateTmrCall->getArg(3);
        const FunctionDecl *CallbackDecl = unwrapCallbackArg(CallbackArg);
        if (CallbackDecl == nullptr) return;

        MyWriter->writeFunction(CallbackDecl, Result);
    }
}

schmeller::ROS2DepCheck::CreateTmrHandler::CreateTmrHandler(DataWriter *Writer) {
    MyWriter = Writer;
}