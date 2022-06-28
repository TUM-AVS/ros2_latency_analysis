#include "MemberRefCollector.hh"

void MemberRefCollector::run(const MatchFinder::MatchResult &Result) {
    if (const CallExpr *CreateSubCall = Result.Nodes.getNodeAs<clang::CallExpr>("create_sub_call")) {
        if (CreateSubCall->getNumArgs() < 3) {
            std::cout << "Expression has " << CreateSubCall->getNumArgs() << " args, expected 3 or more. Ignoring."
                      << std::endl;
            return;
        }
        const Expr *CallbackArg = CreateSubCall->getArg(2);
        const Expr *CallbackBody = this->unwrapCallbackArg(CallbackArg);
        if (CallbackBody == nullptr) return;

        std::vector<MemberExpr *> CallbackMemberRefs;
        this->getCallbackMemberRefs((Expr *) CallbackBody, CallbackMemberRefs);
        for (MemberExpr *MemberRef: CallbackMemberRefs) {
            std::cout << MemberRef->getMemberNameInfo().getAsString()
                    << "---"
                    << " L:" << MemberRef->isLValue()
                    << " GL:" << MemberRef->isGLValue()
                    << " PR:" << MemberRef->isPRValue()
                    << " X:" << MemberRef->isXValue()
                    << std::endl;
        }

        std::cout << std::endl;
        std::cout << std::endl;
    }
}

const Expr *MemberRefCollector::unwrapCallbackArg(const Expr *CallbackArg) {
    if (!checkClass(CallbackArg, Stmt::StmtClass::MaterializeTemporaryExprClass,
                    "Callback arg wrapper is not a temporary expression"))
        return nullptr;

    // Get Bind call
    const Expr *TheChild = getChild(CallbackArg, 0, 1);
    if (!checkClass(TheChild, Stmt::StmtClass::CallExprClass,
                    "Callback arg is not a bind call")) {
        if (!checkClass(TheChild, Stmt::StmtClass::LambdaExprClass,
                        "  and also not a lambda")) return nullptr;

        return TheChild; // Child is a LambdaExpr and is its own function body
    }

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

    const Stmt *CallbackDef = this->getCallbackDefinition(static_cast<const DeclRefExpr *>(TheChild));
    return static_cast<const Expr *>(CallbackDef);
}

const Stmt *MemberRefCollector::getCallbackDefinition(const DeclRefExpr *CallbackSignature) {
    if (!checkClass(CallbackSignature, Stmt::StmtClass::DeclRefExprClass,
                    "Callback signature is null"))
        return nullptr;

    const ValueDecl *CallbackDecl = CallbackSignature->getDecl();
    if (CallbackDecl == nullptr || CallbackDecl->getKind() != Decl::CXXMethod) return nullptr;
    const CXXMethodDecl *CXXDecl = dynamic_cast<const CXXMethodDecl *>(CallbackDecl->getAsFunction());
    return CXXDecl->getBody();
}

void MemberRefCollector::getCallbackMemberRefs(Expr *Expression, std::vector<MemberExpr *> &Accumulator) {
    if (Expression->getStmtClass() == Stmt::StmtClass::MemberExprClass) {
        Accumulator.push_back(static_cast<MemberExpr *>(Expression));
        return;
    }

    for (Stmt *Child: Expression->children())
        this->getCallbackMemberRefs(static_cast<Expr *>(Child), Accumulator);
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