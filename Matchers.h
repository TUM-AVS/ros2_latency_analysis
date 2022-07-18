//
// Created by max on 08.07.22.
//

#ifndef ROS2_INTERNAL_DEPENDENCY_CHECKER_MATCHERS_H
#define ROS2_INTERNAL_DEPENDENCY_CHECKER_MATCHERS_H

#define FunctionContainingXMatcher(x) functionDecl(hasAnyBody(hasDescendant(x)))

#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>

using namespace clang;
using namespace clang::ast_matchers;

namespace schmeller::ROS2DepCheck::Matchers {
  extern const char *MemberReference;
  extern const char *OpWithLhsMember;
  extern const char *OpWithRhsMember;
  extern const char *DeclUsingMember;
  extern const char *CallToMember;
  extern const char *CallWithMemberAsArg;
  extern const char *PublishOfMemberPublisher;

  extern DeclarationMatcher NodeDeclMatcher;

  extern StatementMatcher MethodBodyMatcher;

  // Matches all references to members declared in a ROS node
  extern internal::BindableMatcher<Stmt> MemberMatcher;

  // Matches (reflexive-transitively) all references to members of members
  // declared in a ROS node
  extern StatementMatcher ChainedMemberMatcher;

  // Matches ROS node members in the LHS of assignments
  extern StatementMatcher MemberIsLHSMatcher;

  // Matches ROS node members in the RHS of assignments
  extern StatementMatcher MemberIsRHSMatcher;

  extern DeclarationMatcher MemberInDeclMatcher;

  // Matches calls to member functions of ROS nodes
  extern StatementMatcher MemberIsCalleeMatcher;

  // Matches ROS node members that are arguments to function calls
  // TODO: matches CXX operators like ->, =, !=
  extern StatementMatcher MemberIsCallArgMatcher;

  extern StatementMatcher AnyPublishCallMatcher;

  // Matches publish calls of members of ROS nodes.
  // TODO: "publish" is string-matched, there is no check whether the publish call
  // is on a rclcpp:Publisher instance
  extern StatementMatcher MemberPublishCallMatcher;

  // Matches all functions containing a call to publish of a ROS node
  extern DeclarationMatcher FuncContainingPubCallMatcher;

  // Matches all definitions of subscription callbacks (no lambdas)
  extern StatementMatcher CallbackFuncMatcher;

  // Matches subscription callbacks that are lambdas
  extern StatementMatcher CallbackLambdaMatcher;

  // Matches all calls to timer/subscription callback registrations
  extern StatementMatcher CallbackRegistrationMatcher;

  extern StatementMatcher PublisherRegistrationMatcher;
} // namespace schmeller::ROS2DepCheck::Matchers
#endif // ROS2_INTERNAL_DEPENDENCY_CHECKER_MATCHERS_H
