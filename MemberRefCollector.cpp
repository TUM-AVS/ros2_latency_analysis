#include "MemberRefCollector.hh"

void MemberRefCollector::run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr *expr = Result.Nodes.getNodeAs<clang::CallExpr>("create_sub_call")) {
        if (expr->getNumArgs() < 3) {
            std::cout << "Expression has " << expr->getNumArgs() << " args, expected 3 or more. Ignoring." << std::endl;
            return;
        }
        const Expr *CallbackArg = expr->getArg(2);
        const DeclRefExpr *CallbackSig = this->unwrapCallbackArg(CallbackArg);
        this->getCallbackDefinition(CallbackSig);

        std::cout << std::endl;
        std::cout << std::endl;
    }
}

const DeclRefExpr *MemberRefCollector::unwrapCallbackArg(const Expr *CallbackArg) {
    if (!checkClass(CallbackArg, Stmt::StmtClass::MaterializeTemporaryExprClass,
                    "Callback arg wrapper is not a temporary expression"))
        return nullptr;

    // Get Bind call
    const Expr *TheChild = getChild(CallbackArg, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::CallExprClass,
                    "Callback arg is not a bind call")) {
        TheChild->dumpColor();
    }
    return nullptr;

    // Get 1st arg
    TheChild = getChild(TheChild, 1, 4);
    if (!checkClass(TheChild, Stmt::StmtClass::MaterializeTemporaryExprClass,
                    "1st arg of bind call has unexpected type"))
        return nullptr;

    // Get &(...) expr
    TheChild = getChild(TheChild, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::UnaryOperatorClass,
                    "Bind arg is not a function reference operator"))
        return nullptr;

    TheChild = getChild(TheChild, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::DeclRefExprClass,
                    "Reference operator arg is not a decl ref"))
        return nullptr;

    return static_cast<const DeclRefExpr *>(TheChild);
}

void MemberRefCollector::getCallbackDefinition(const DeclRefExpr *CallbackSignature) {
    if (!checkClass(CallbackSignature, Stmt::StmtClass::DeclRefExprClass,
                    "Callback signature is null"))
        return;

    const ValueDecl *CallbackDecl = CallbackSignature->getDecl();
    if (CallbackDecl == nullptr || CallbackDecl->getKind() != Decl::CXXMethod) return;
    const CXXMethodDecl *CXXDecl = dynamic_cast<const CXXMethodDecl *>(CallbackDecl->getAsFunction());
    CXXDecl->getBody()->dumpColor();
}

void MemberRefCollector::getCallbackMemberRefs(const Expr *CallbackBody) {

}

const Expr *getChild(const Expr *Parent, int ChildIndex, int ExpectedNChildren) {
    int ChildCount = 0;
    const Stmt *TheChild;
    for (const Stmt *Child: Parent->children()) {
        if (ChildCount == ChildIndex) TheChild = Child;
        ChildCount++;
    }

    if (ChildCount != ExpectedNChildren) return nullptr;
    return static_cast<const Expr *>(TheChild);
}

bool checkClass(const Expr *Expression, const Stmt::StmtClass &ExpectedClass, const std::string &NotMatchingMessage) {
    if (Expression == nullptr || Expression->getStmtClass() != ExpectedClass) {
        std::cout << NotMatchingMessage << std::endl;
        return false;
    }

    return true;
}