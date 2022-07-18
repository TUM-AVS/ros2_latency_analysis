#include "Matchers.h"

namespace schmeller::ROS2DepCheck::Matchers {
const char *MemberReference = "ref";
const char *OpWithLhsMember = "op_with_lhs_member";
const char *OpWithRhsMember = "op_with_rhs_member";
const char *DeclUsingMember = "decl_using_member";
const char *CallToMember = "call_to_member";
const char *CallWithMemberAsArg = "call_with_member_as_arg";
const char *PublishOfMemberPublisher = "publish_of_member_publisher";

DeclarationMatcher NodeDeclMatcher =
    cxxRecordDecl(hasAnyBase(hasType(cxxRecordDecl(hasName("rclcpp::Node")))))
        .bind("node_decl");

StatementMatcher MethodBodyMatcher = compoundStmt(forCallable(
    cxxMethodDecl(hasDeclContext(NodeDeclMatcher)).bind("containing_method")));

// Matches all references to members declared in a ROS node
internal::BindableMatcher<Stmt> MemberMatcher =
    memberExpr(hasDeclaration(hasAncestor(NodeDeclMatcher)),
               optionally(hasAncestor(MethodBodyMatcher)));

// Matches (reflexive-transitively) all references to members of members
// declared in a ROS node
StatementMatcher ChainedMemberMatcher =
    anyOf(MemberMatcher.bind(MemberReference),
          memberExpr(hasDescendant(MemberMatcher)).bind(MemberReference));

// Matches ROS node members in the LHS of assignments
StatementMatcher MemberIsLHSMatcher =
    binaryOperation(isAssignmentOperator(), hasLHS(ChainedMemberMatcher))
        .bind(OpWithLhsMember);

// Matches ROS node members in the RHS of assignments
StatementMatcher MemberIsRHSMatcher =
    binaryOperation(isAssignmentOperator(), hasRHS(ChainedMemberMatcher))
        .bind(OpWithRhsMember);

DeclarationMatcher MemberInDeclMatcher =
    varDecl(has(ChainedMemberMatcher)).bind(DeclUsingMember);

// Matches calls to member functions of ROS nodes
StatementMatcher MemberIsCalleeMatcher =
    callExpr(callee(ChainedMemberMatcher)).bind(CallToMember);

// Matches ROS node members that are arguments to function calls
// TODO: matches CXX operators like ->, =, !=
StatementMatcher MemberIsCallArgMatcher =
    callExpr(hasAnyArgument(ChainedMemberMatcher)).bind(CallWithMemberAsArg);

StatementMatcher AnyPublishCallMatcher =
    cxxMemberCallExpr(callee(functionDecl(hasName("publish"))))
        .bind(PublishOfMemberPublisher);

// Matches publish calls of members of ROS nodes.
// TODO: "publish" is string-matched, there is no check whether the publish call
// is on a rclcpp:Publisher instance
StatementMatcher MemberPublishCallMatcher =
    memberExpr(
        hasDeclaration(fieldDecl(hasAncestor(NodeDeclMatcher)).bind("decl")),
        hasAncestor(AnyPublishCallMatcher), hasAncestor(MethodBodyMatcher))
        .bind(MemberReference);

// Matches all functions containing a call to publish of a ROS node
DeclarationMatcher FuncContainingPubCallMatcher =
    FunctionContainingXMatcher(MemberPublishCallMatcher);

// Matches all definitions of subscription callbacks (no lambdas)
StatementMatcher CallbackFuncMatcher =
    callExpr(callee(functionDecl(anyOf(hasName("create_timer"),
                                       hasName("create_subscription")))),
             optionally(hasArgument(0, stringLiteral().bind("topic_name"))),
             hasAnyArgument(hasDescendant(declRefExpr(hasDeclaration(
                 functionDecl(hasAncestor(MethodBodyMatcher)).bind("decl"))))))
        .bind("call");

// Matches subscription callbacks that are lambdas
StatementMatcher CallbackLambdaMatcher =
    callExpr(callee(functionDecl(anyOf(hasName("create_timer"),
                                       hasName("create_subscription")))),
             optionally(hasArgument(0, stringLiteral().bind("topic_name"))),
             hasDescendant(lambdaExpr().bind("decl")))
        .bind("call");

// Matches all calls to timer/subscription callback registrations
StatementMatcher CallbackRegistrationMatcher =
    callExpr(callee(functionDecl(anyOf(hasName("create_timer"),
                                       hasName("create_subscription")))))
        .bind("call");

StatementMatcher PublisherRegistrationMatcher = binaryOperation(
    isAssignmentOperator(), hasLHS(ChainedMemberMatcher),
    hasRHS(callExpr(
        callee(functionDecl(hasName("create_publisher"))),
        optionally(hasArgument(0, stringLiteral().bind("topic_name"))))));
} // namespace schmeller::ROS2DepCheck::Matchers