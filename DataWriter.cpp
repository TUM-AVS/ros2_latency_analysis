//
// Created by max on 30.06.22.
//

#include "DataWriter.h"

void schmeller::ROS2DepCheck::DataWriter::writeFunction(const FunctionDecl *Decl, const MatchFinder::MatchResult &Result) {
    OutStream << lvl(0) << "- " << Decl->getID() << ":\n";
    OutStream << lvl(2) << "type: function\n";
    OutStream << lvl(2) << "qualified_name: " << Decl->getQualifiedNameAsString() << '\n';
    OutStream << lvl(2) << "source_range: " << Decl->getSourceRange().printToString(*Result.SourceManager) << '\n';
    OutStream << lvl(2) << "member_refs:" << '\n';

    std::vector<MemberExpr *> CallbackMemberRefs;
    getCallbackMemberRefs((Expr *) getCallbackBody(Decl), CallbackMemberRefs);
    for (MemberExpr *MemberRef: CallbackMemberRefs) {
        OutStream << lvl(3) << "- " << MemberRef->getMemberDecl()->getID() << ":\n";
        OutStream << lvl(5) << "source_range:" << MemberRef->getSourceRange().printToString(*Result.SourceManager) << '\n';
        OutStream << lvl(5) << "is_lvalue: " << MemberRef->isLValue() << '\n';
        OutStream << lvl(5) << "is_glvalue: " << MemberRef->isGLValue() << '\n';
        OutStream << lvl(5) << "is_prvalue: " << MemberRef->isPRValue() << '\n';
        OutStream << lvl(5) << "is_xvalue: " << MemberRef->isXValue() << '\n';
        OutStream << lvl(5) << "is_bound_member_function: " << MemberRef->isBoundMemberFunction(*Result.Context) << '\n';
        OutStream << lvl(5) << "qualified_name: " << MemberRef->getMemberDecl()->getQualifiedNameAsString() << '\n';
    }

    OutStream << std::endl;
}

void schmeller::ROS2DepCheck::DataWriter::writeLambda(const LambdaExpr *Decl, const MatchFinder::MatchResult &Result) {
    writeFunction(Decl->getAsBuiltinConstantDeclRef(*Result.Context)->getAsFunction(), Result);
}

void schmeller::ROS2DepCheck::DataWriter::writeNode(const CXXRecordDecl *Decl, const MatchFinder::MatchResult &Result) {
    OutStream << lvl(0) << "- " << Decl->getID() << ":\n";
    OutStream << lvl(2) << "type: node\n";
    OutStream << lvl(2) << "qualified_name: " << Decl->getQualifiedNameAsString() << '\n';
    OutStream << lvl(2) << "source_range: " << Decl->getSourceRange().printToString(*Result.SourceManager) << '\n';

    OutStream << lvl(2) << "fields:" << '\n';
    for (const auto* Field : Decl->fields()) {
        OutStream << lvl(3) << "- " << Field->getID() << ":\n";
        OutStream << lvl(5) << "qualified_name: " << Field->getQualifiedNameAsString() << '\n';
        OutStream << lvl(5) << "source_range: " << Field->getSourceRange().printToString(*Result.SourceManager) << '\n';
    }

    OutStream << lvl(2) << "methods:" << '\n';
    for (const auto* Method : Decl->methods()) {
        OutStream << lvl(3) << "- " << Method->getID() << ":\n";
        OutStream << lvl(5) << "qualified_name: " << Method->getQualifiedNameAsString() << '\n';
        OutStream << lvl(5) << "source_range: " << Method->getSourceRange().printToString(*Result.SourceManager) << '\n';
    }

    OutStream << std::endl;
}

std::string schmeller::ROS2DepCheck::DataWriter::lvl(int Level) {
    return std::string(Level * 2, ' ');
}
