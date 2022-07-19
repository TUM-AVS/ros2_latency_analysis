//
// Created by max on 30.06.22.
//

#include "DataWriter.h"

namespace schmeller::ROS2DepCheck {
json DataWriter::functionDeclToJson(const FunctionDecl *Decl,
                                    const MatchFinder::MatchResult &Result) {
  json J = {
      {"id", Decl->getID()},
      {"qualified_name", Decl->getQualifiedNameAsString()},
      {"source_range", sourceRangeToJson(Decl->getSourceRange(), Result)},
      {"signature",
       {{"return_type", Decl->getReturnType().getCanonicalType().getAsString()},
        {"parameter_types", json::array()}}}};

  json *ParamTypes = &(J["signature"]["parameter_types"]);
  for (auto *Param : Decl->parameters()) {
    ParamTypes->push_back(Param->getType().getCanonicalType().getAsString());
  }

  return J;
}

json DataWriter::nodeToJson(const CXXRecordDecl *Decl,
                            const MatchFinder::MatchResult &Result) {
  json J = {{"id", Decl->getID()},
            {"qualified_name", Decl->getQualifiedNameAsString()},
            {"source_range", sourceRangeToJson(Decl->getSourceRange(), Result)},
            {"fields", json::array()},
            {"methods", json::array()}};

  for (const auto *Field : Decl->fields()) {
    J["fields"].push_back(
        {{"id", Field->getID()},
         {"qualified_name", Field->getQualifiedNameAsString()},
         {"source_range", sourceRangeToJson(Field->getSourceRange(), Result)}});
  }

  for (const auto *Method : Decl->methods()) {
    J["methods"].push_back(
        {{"id", Method->getID()},
         {"qualified_name", Method->getQualifiedNameAsString()},
         {"source_range",
          sourceRangeToJson(Method->getSourceRange(), Result)}});
  }

  return J;
}

json DataWriter::memberChainToJson(const std::vector<MemberExpr *> &MemberChain,
                                   const MatchFinder::MatchResult &Result) {
  json J = json::array();
  for (const MemberExpr *M : MemberChain) {
    J.push_back(memberToJson(M, Result));
  }

  return J;
}

json DataWriter::memberToJson(const MemberExpr *Member,
                              const MatchFinder::MatchResult &Result) {
  return {
      {"id", Member->getMemberDecl()->getID()},
      {"qualified_name", Member->getMemberDecl()->getQualifiedNameAsString()},
      {"source_range", sourceRangeToJson(Member->getSourceRange(), Result)},
      {"name", Member->getMemberNameInfo().getAsString()}};
}

json DataWriter::sourceRangeToJson(const SourceRange &Range,
                                   const MatchFinder::MatchResult &Result) {
  return {{"begin", Range.getBegin().printToString(*Result.SourceManager)},
          {"end", Range.getEnd().printToString(*Result.SourceManager)}};
}

void DataWriter::write(const char *Filename) {
  std::ofstream OutStream(Filename);
  OutStream << std::setw(4) << Json << std::endl;
  OutStream.close();

  Json = {};
}

void DataWriter::addAtPath(const char *Path, const json &Node) {
  std::vector<std::string> PathSegments;
  std::stringstream PathStream(Path);
  std::string PathSegment;

  while (std::getline(PathStream, PathSegment, '/'))
    PathSegments.push_back(PathSegment);

  json *JsonRef = &Json;
  for (auto It = PathSegments.begin(); It != PathSegments.end(); ++It) {
    if (!JsonRef->contains(Path)) {
      if (It + 1 == PathSegments.end())
        (*JsonRef)[*It] = json::array(); // Last node in path is array
      else
        (*JsonRef)[*It] = json::object(); // All other nodes are objects
    }

    JsonRef = &((*JsonRef)[*It]);
  }

  JsonRef->push_back(Node);
}
} // namespace schmeller::ROS2DepCheck