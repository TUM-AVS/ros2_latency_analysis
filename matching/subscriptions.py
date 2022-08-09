import pickle
import re
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Iterable, Set, Tuple

from bidict import bidict
from termcolor import colored

sys.path.append("../../autoware/build/tracetools_read/")
sys.path.append("../../autoware/build/tracetools_analysis/")

from clang_interop.cl_types import ClMethod, ClContext, ClSubscription
from tracing_interop.tr_types import TrContext, TrSubscriptionObject, TrSubscription, TrCallbackSymbol, TrTimer


class TKind(Enum):
    # language=PythonRegExp
    identifier = r"(?P<identifier>(?:[\w$-]+::)*[\w$-]+|[+-]?[0-9]+|\".*?\"|'.*?')"
    # language=PythonRegExp
    ang_open = r"(?P<ang_open><)"
    # language=PythonRegExp
    ang_close = r"(?P<ang_close>>)"
    # language=PythonRegExp
    par_open = r"(?P<par_open>\()"
    # language=PythonRegExp
    par_close = r"(?P<par_close>\))"
    # language=PythonRegExp
    hash = r"(?P<hash>#)"
    # language=PythonRegExp
    curl_open = r"(?P<curl_open>\{)"
    # language=PythonRegExp
    curl_close = r"(?P<curl_close>})"
    # language=PythonRegExp
    brack_open = r"(?P<brack_open>\[)"
    # language=PythonRegExp
    brack_close = r"(?P<brack_close>])"
    # language=PythonRegExp
    whitespace = r"(?P<whitespace>\s+)"
    # language=PythonRegExp
    ref = r"(?P<ref>&)"
    # language=PythonRegExp
    ptr = r"(?P<ptr>\*)"
    # language=PythonRegExp
    comma = r"(?P<comma>,)"
    # language=PythonRegExp
    ns_sep = r"(?P<ns_sep>::)"
    # language=PythonRegExp
    unknown_symbol = r"(?P<unknown_symbol>\?)"

    def __repr__(self):
        return self.name


class ASTEntry:
    def get_token_stream(self):
        pass


@dataclass
class ASTLeaf(ASTEntry):
    kind: TKind
    spelling: str

    def get_token_stream(self):
        return [self]

    def __repr__(self):
        return self.spelling if self.kind != TKind.identifier else f'"{self.spelling}"'


@dataclass
class ASTNode(ASTEntry):
    type: str
    children: List[ASTEntry] = field(default_factory=list)
    parent: Optional['ASTNode'] = field(default_factory=lambda: None)
    start: ASTLeaf | None = field(default_factory=lambda: None)
    end: ASTLeaf | None = field(default_factory=lambda: None)

    def get_token_stream(self) -> List[ASTLeaf]:
        stream = []
        for c in self.children:
            stream += c.get_token_stream()
        if self.start:
            stream.insert(0, self.start)
        if self.end:
            stream.append(self.end)
        return stream

    def __repr__(self):
        tokens = self.get_token_stream()
        ret = ""
        last_tkind = None
        for t in tokens:
            match t.kind:
                case TKind.identifier:
                    if last_tkind == TKind.identifier:
                        ret += " "
            ret += t.spelling
            last_tkind = t.kind
        return ret


BRACK_MAP = bidict({
        TKind.curl_open: TKind.curl_close,
        TKind.par_open: TKind.par_close,
        TKind.brack_open: TKind.brack_close,
        TKind.ang_open: TKind.ang_close
})

BRACK_SPELLING_MAP = bidict({
        '{': '}',
        '(': ')',
        '[': ']',
        '<': '>'
})

TR_BLACKLIST = [
        "tf2_ros::TransformListener",
        "rtc_auto_approver::RTCAutoApproverInterface",
        "rclcpp::TimeSource",
        "rclcpp::ParameterService",
        "rclcpp_components::ComponentManager",
        "rosbag2",
        "std_srvs::srv"
]


def cl_deps_to_tr_deps(matches: Set[Tuple], tr: TrContext, cl: ClContext):

    ##################################################
    # Narrow down matches
    ##################################################
    # The `match()` function returns an n-to-m
    # mapping between cl and tr symbols.
    # This n-to-m mapping has to be narrowed down
    # to 1-to-1. This is done by building cohorts
    # of cl symbols which belong to the same node
    # and filtering out outliers.
    ##################################################

    final_tr_deps = dict()
    for cl_cb_id, cl_dep_ids in cl.dependencies.items():
        cl_cb_id: int
        cl_dep_ids: Iterable[int]

        ##################################################
        # 1. For all cl dependencies, build all possible
        #    tr dependencies with the matches we have.
        ##################################################

        cl_cb = next(filter(lambda m: m.id == cl_cb_id, cl.methods), None)
        cl_deps = set(filter(lambda m: m.id in cl_dep_ids, cl.methods))

        if cl_cb is None or len(cl_deps) < len(cl_deps):
            print(colored(f"[ERROR][CL] Callback has not all CL methods defined", "red"))
        if cl_cb is None:
            continue  # for cl.dependencies.items()

        # Because the mapping is n-to-m, we have tr_cbs as a set, instead of a single tr_cb
        tr_cbs = set(tr_obj for cl_obj, tr_obj, *_ in matches if cl_obj == cl_cb)
        tr_deps = set(tr_obj for cl_obj, tr_obj, *_ in matches if cl_obj in cl_deps)

        ##################################################
        # 2. Filter out all combinations where
        #    dependencies leave a node.
        ##################################################

        def owner_node(sym: TrCallbackSymbol):
            cb_objs = sym.callback_objs
            owners = [cb_obj.owner for cb_obj in cb_objs]
            owner_nodes = set()
            for owner in owners:
                match owner:
                    case TrSubscriptionObject() as sub:
                        sub: TrSubscriptionObject
                        owner_nodes.add(sub.subscription.node)
                    case TrTimer() as tmr:
                        tmr: TrTimer
                        owner_nodes.update(tmr.nodes)

            if len(owner_nodes) == 1:
                return owner_nodes.pop()

            return None

        viable_matchings = {}
        for tr_cb in tr_cbs:
            tr_cb: TrCallbackSymbol
            owner = owner_node(tr_cb)
            if owner is None:
                continue  # for tr_cbs

            valid_deps = set(dep for dep in tr_deps if owner_node(dep) == owner)
            if not valid_deps:
                continue  # for tr_cbs

            viable_matchings[tr_cb] = valid_deps

        if not viable_matchings:
            print(colored(f"[ERROR][CL] Callback has not all TR equivalents for CL: {cl_cb.signature}", "red"))
            continue  # for cl.dependencies.items()

        ##################################################
        # 3. Select the matching with the highest number
        #    of mapped dependencies (= the smallest number
        #    of unmapped cl deps)
        ##################################################

        print(len(viable_matchings), ', '.join(map(str, map(len, viable_matchings.values()))))

        final_tr_deps.update(viable_matchings)

    return final_tr_deps


def match(tr: TrContext, cl: ClContext):
    def _is_excluded(symbol: str):
        return any(item in symbol for item in TR_BLACKLIST)

    cl_methods = [cb for cb in cl.methods
                  if any(sub.callback_id == cb.id for sub in cl.subscriptions)
                  or any(tmr.callback_id == cb.id for tmr in cl.timers)]

    tr_callbacks = [(sym.symbol, sym) for sym in tr.callback_symbols.values() if not _is_excluded(sym.symbol)]
    cl_callbacks = [(cb.signature, cb) for cb in cl_methods]

    tr_callbacks = [(repr(sanitize(k)), v) for k, v in tr_callbacks]
    cl_callbacks = [(repr(sanitize(k)), v) for k, v in cl_callbacks]

    matches_sig = set()

    tr_matched = set()
    cl_matched = set()

    for cl_sig, cl_obj in cl_callbacks:
        matches = set(tr_obj for tr_sig, tr_obj in tr_callbacks if tr_sig == cl_sig)
        tr_matched |= matches
        if matches:
            cl_matched.add(cl_obj)
        for tr_obj in matches:
            matches_sig.add((cl_obj, tr_obj, cl_sig))

    matches_topic = set()
    for _, cl_obj in cl_callbacks:
        # Get subscription of the callback (if any)
        cl_sub: ClSubscription | None = next((sub for sub in cl.subscriptions if sub.callback_id == cl_obj.id), None)

        if not cl_sub:
            continue

        cl_topic = re.sub(r"~/(input/)?", "", cl_sub.topic)

        matches = set()
        for _, tr_obj in tr_callbacks:
            tr_cb = tr_obj.callback_objs[0] if len(tr_obj.callback_objs) == 1 else None
            if not tr_cb:
                continue

            match tr_cb.owner:
                case TrSubscriptionObject(subscription=tr_sub):
                    tr_sub: TrSubscription
                    tr_topic = tr_sub.topic_name
                    if not tr_topic:
                        continue
                case _:
                    continue

            if tr_topic.endswith(cl_topic):
                matches_topic.add((cl_obj, tr_obj, cl_topic, tr_topic))
                matches.add(tr_obj)

        tr_matched |= matches
        if matches:
            cl_matched.add(cl_obj)

    all_matches = matches_sig | matches_topic

    def count_dup(matches):
        cl_dup = 0
        tr_dup = 0
        for (cl_obj, tr_obj, *_) in matches:
            n_cl_dups = len([cl2 for cl2, *_ in matches if cl2 == cl_obj])
            if n_cl_dups > 1:
                cl_dup += 1 / n_cl_dups

            n_tr_dups = len([tr2 for _, tr2, *_ in matches if tr2 == tr_obj])
            if n_tr_dups > 1:
                tr_dup += 1 / n_tr_dups

        print(int(cl_dup), int(tr_dup))

    count_dup(all_matches)

    tr_unmatched = set(tr_obj for _, tr_obj in tr_callbacks) - tr_matched
    cl_unmatched = set(cl_obj for _, cl_obj in cl_callbacks) - cl_matched

    return all_matches, tr_unmatched, cl_unmatched


def match_and_modify_children(node: ASTEntry, match_func):
    if not isinstance(node, ASTNode):
        return node

    for i in range(len(node.children)):
        seq_head = node.children[:i]
        seq_tail = node.children[i:]
        match_result = match_func(seq_head, seq_tail, node)
        if match_result is not None:
            node.children = match_result

    return node


def sanitize(sig: str):
    ast = build_ast(sig)

    def _remove_qualifiers(node: ASTEntry):
        match node:
            case ASTLeaf(TKind.identifier, 'class' | 'struct' | 'const'):
                return None
        return node

    def _remove_std_wrappers(node: ASTEntry):
        def _child_seq_matcher(head, tail, _):
            match tail:
                case [ASTLeaf(TKind.identifier, "std::allocator"), ASTNode('<>'), *rest]:
                    return head + rest
                case [ASTLeaf(TKind.identifier, "std::shared_ptr"), ASTNode('<>', ptr_type), *rest]:
                    return head + ptr_type + rest
            return None

        return match_and_modify_children(node, _child_seq_matcher)

    def _remove_std_bind(node: ASTEntry):
        def _child_seq_matcher(head, tail, parent):
            match tail:
                case [ASTLeaf(TKind.identifier, "std::_Bind"),
                      ASTNode(type='<>', children=[
                          callee_ret,
                          ASTNode('()', children=[*callee_ptr, ASTNode('()', bind_args)]),
                          ASTNode('()') as replacement_args])]:

                    return [callee_ret] + head + [ASTNode('()', callee_ptr, parent,
                                                          ASTLeaf(TKind.par_open, '('),
                                                          ASTLeaf(TKind.par_close, ')')),
                                                  replacement_args]
            return None

        return match_and_modify_children(node, _child_seq_matcher)

    def _unwrap_lambda(node: ASTEntry):
        def _child_seq_matcher(head, tail, parent):
            match tail:
                case [ASTNode(type='()') as containing_method_args,
                      ASTLeaf(TKind.ns_sep),
                      ASTNode(type='{}',
                              children=[
                                  ASTLeaf(TKind.identifier, "lambda"),
                                  ASTNode(type='()') as lambda_sig,
                                  ASTLeaf(TKind.hash),
                                  ASTLeaf(TKind.identifier)
                              ]),
                      *_]:
                    return [ASTLeaf(TKind.identifier, "void")] + \
                           [ASTNode('()',
                                    head + [containing_method_args],
                                    parent,
                                    ASTLeaf(TKind.par_open, '('),
                                    ASTLeaf(TKind.par_close, ')'))] + \
                           [lambda_sig] + tail[3:]

            return None

        return match_and_modify_children(node, _child_seq_matcher)

    def _remove_artifacts(node: ASTEntry):
        def _child_seq_matcher(head, tail, _):
            match tail:
                case [ASTLeaf(TKind.ns_sep), ASTLeaf(TKind.ref | TKind.ptr), *rest]:
                    return head + rest
            return None

        match node:
            case ASTLeaf(TKind.identifier, spelling):
                return ASTLeaf(TKind.identifier, re.sub(r"(_|const)$", "", spelling))
            case ASTNode('<>', []):
                return None
            case ASTNode():
                return match_and_modify_children(node, _child_seq_matcher)
            case ASTLeaf(TKind.ref | TKind.ptr | TKind.ns_sep | TKind.unknown_symbol):
                return None
        return node

    def _replace_verbose_types(node: ASTEntry):
        match node:
            case ASTLeaf(TKind.identifier, "_Bool"):
                return ASTLeaf(TKind.identifier, "bool")

        return node

    def _replace_lambda_enumerations(node: ASTEntry):
        match node:
            case ASTNode(children=[*_, ASTLeaf(TKind.identifier, idf)]) as node:
                if re.fullmatch(r"\$_[0-9]+", idf):
                    node.children = node.children[:-1] + [ASTLeaf(TKind.identifier, "$lambda")]

        return node

    def _remove_return_types(node: ASTEntry):
        match node:
            case ASTNode("ast", [ASTLeaf(TKind.identifier), qualified_name, ASTNode('()') as params]) as node:
                match qualified_name:
                    case ASTNode('()', name_unwrapped):
                        qualified_name = name_unwrapped
                    case _:
                        qualified_name = [qualified_name]
                node.children = qualified_name + [params]

        return node

    ast = traverse(ast, _remove_qualifiers)
    ast = traverse(ast, _remove_std_wrappers)
    ast = traverse(ast, _remove_std_bind)
    ast = traverse(ast, _unwrap_lambda)
    ast = traverse(ast, _remove_artifacts)
    ast = traverse(ast, _replace_verbose_types)
    ast = traverse(ast, _replace_lambda_enumerations)
    #ast = _remove_return_types(ast)
    return ast


def traverse(node: ASTEntry, action) -> ASTEntry | None:
    match node:
        case ASTNode():
            children = []
            for c in node.children:
                c = traverse(c, action)
                match c:
                    case list():
                        children += c
                    case None:
                        pass
                    case _:
                        children.append(c)

            node.children = children
    return action(node)


def build_ast(sig: str):
    tokens = tokenize(sig)

    ast = ASTNode("ast", [], None)
    parens_stack = []
    current_node = ast
    for token in tokens:
        match token.kind:
            case TKind.ang_open | TKind.curl_open | TKind.brack_open | TKind.par_open:
                parens_stack.append(token.kind)
                brack_content_ast_node = ASTNode(f"{token.spelling}{BRACK_SPELLING_MAP[token.spelling]}",
                                                 [],
                                                 current_node,
                                                 start=token,
                                                 end=ASTLeaf(BRACK_MAP[token.kind], BRACK_SPELLING_MAP[token.spelling]))
                current_node.children.append(brack_content_ast_node)
                current_node = brack_content_ast_node
            case TKind.ang_close | TKind.curl_close | TKind.brack_close | TKind.par_close:
                if not parens_stack or BRACK_MAP.inv[token.kind] != parens_stack[-1]:
                    expect_str = parens_stack[-1] if parens_stack else "nothing"
                    raise ValueError(
                            f"Invalid brackets: encountered {token.spelling} when expecting {expect_str} in '{sig}'")
                parens_stack.pop()
                current_node = current_node.parent
            case TKind.whitespace:
                continue
            case _:
                current_node.children.append(token)

    if parens_stack:
        raise ValueError(f"Token stream finished but unclosed brackets remain: {parens_stack} in '{sig}'")

    return ast


def tokenize(sig: str) -> List[ASTLeaf]:
    token_matchers = [t.value for t in TKind]
    tokens = list(re.finditer('|'.join(token_matchers), sig))

    prev_end = 0
    for t in tokens:
        t_start, t_end = t.span()
        if t_start != prev_end:
            raise ValueError(f"Tokenizer failed at char {t_start}: '{sig}'")
        prev_end = t_end

    if prev_end != len(sig):
        raise ValueError(f"Tokenization not exhaustive for: '{sig}'")

    tokens = [tuple(next(filter(lambda pair: pair[-1] is not None, t.groupdict().items()))) for t in tokens]
    tokens = [ASTLeaf(TKind.__members__[k], v) for k, v in tokens]
    return tokens


if __name__ == "__main__":
    with open("../cache/cl_objects_7b616c9c48.pkl", "rb") as f:
        print("Loading Clang Objects... ", end='')
        cl: ClContext = pickle.load(f)
        print("Done.")

    with open("../cache/tr_objects_c1e0d50b8d.pkl", "rb") as f:
        print("Loading Tracing Objects... ", end='')
        tr: TrContext = pickle.load(f)
        print("Done.")

    matches, _, _ = match(tr, cl)
    cl_deps_to_tr_deps(matches, tr, cl)
