import clang.cindex as ci
from clang.cindex import TokenKind as tk
from clang.cindex import CursorKind as ck
from typing import List


class TUHandler:
    BRACK_MAP = {
        ')': '(',
        ']': '[',
        '>': '<',
        '}': '{'
    }

    def __init__(self, tu: ci.TranslationUnit):
        self.tu = tu

    def get_subscription_callback_handlers(self):
        #################################################
        # Find create_subscription function calls
        #################################################

        c: ci.Cursor = self.tu.cursor
        create_subscription_tokens = [
            " ".join(map(lambda t2: t2.spelling, t.cursor.get_tokens()))
            for t in c.get_tokens() 
            if t.kind == tk.IDENTIFIER  and t.spelling == "create_subscription"
            ]

        print(create_subscription_tokens)
        
        #################################################
        # Extract callback function identifier
        #################################################


        #################################################
        # Locate definition for callback function
        #################################################
        pass

    def get_timer_callback_handlers(self):
        pass

    def get_member_accesses(self, function_def: ci.Cursor):
        pass

    # def consume_generics(ts: List[ci.Token]):
    #     if not ts or ts[0].spelling != '<':
    #         return ts, None

    #     gen = []
    #     for i, t in enumerate(ts):
    #         match t.spelling:
    #             case '<':
    #                 pass
    #             case '>':
    #                 return ts[i+1:], gen
    #             case _:
    #                 gen.append(t)

    #     return ts, None

    # def consume_args(ts: List[ci.Token]):
    #     if not ts or ts[0].spelling != '(':
    #         print(f"Opening token is {ts[0].spelling}, not (")
    #         return ts, None

    #     ts = ts[1:]  # strip start tok

    #     args = []
    #     current_arg = []
    #     brack_depth = 1
    #     for i, t in enumerate(ts):
    #         match t.spelling:
    #             case '(':
    #                 brack_depth += 1
    #                 current_arg.append(t)
    #             case ')':
    #                 brack_depth -= 1
    #                 if brack_depth == 0:
    #                     args.append(current_arg)
    #                     return ts[i+1:], args
    #                 else:
    #                     current_arg.append(t)
    #             case ',':
    #                 if brack_depth > 1:
    #                     current_arg.append(t)
    #                 else:
    #                     args.append(current_arg)
    #                     current_arg = []
    #             case _:
    #                 current_arg.append(t)

    #     return ts, None

    # def consume_function_identifier(ts: List[ci.Token]):
    #     identifier = []
    #     for i, t in enumerate(ts):
    #         match t.kind:
    #             case tk.PUNCTUATION:
    #                 match t.spelling:
    #                     case "(" | "<":
    #                         return ts[i:], identifier
    #                     case _:
    #                         identifier.append(t)
    #             case _:
    #                 identifier.append(t)

    #     return ts, None

    # def consume_function_call(ts: List[ci.Token]):
    #     assert ts and ts[0].kind == tk.IDENTIFIER
    #     ts, identifier = consume_function_identifier(ts)
    #     ts, gen = consume_generics(ts)
    #     ts, args = consume_args(ts)

    #     return ts, identifier, gen, args

    # def find_children(cur: ci.Cursor, find_func):
    #     found = []
    #     if find_func(cur):
    #         found.append(cur)

    #     for c in cur.get_children():
    #         found += find_children(c, find_func)

    #     return found

    # def find_body(cur: ci.Cursor, symbol: List[ci.Token]):
    #     if symbol is None:
    #         return

    #     method_candidates = find_children(cur, lambda c: c.kind == ck.CXX_METHOD and any(
    #         map(lambda t: t.spelling == symbol[-1].spelling, c.get_tokens())))
    #     valid_candidates = []
    #     for cand in method_candidates:
    #         func_bodies = find_children(
    #             cand, lambda c: c.kind == ck.COMPOUND_STMT)
    #         if not func_bodies:
    #             continue
    #         valid_candidates.append(func_bodies[0])

    #     if len(valid_candidates) != 1:
    #         print(
    #             f"Error, {pt(symbol)} has {len(valid_candidates)} candidates for a function definition!")
    #         return None

    #     def _rec(c: ci.Cursor, lvl=0):
    #         print(
    #             f"{' '*lvl*2}{str(c.kind):.<40s} {c.spelling:30s} {';;'.join(str(arg.kind) for arg in c.get_arguments())}")
    #         for ch in c.get_children():
    #             _rec(ch, lvl+1)

    #     _rec(valid_candidates[0])

    #     return list(valid_candidates[0].get_tokens())

    # def get_identifiers_with_lines(tokens: List[ci.Token]):

    #     stmt_extent = (tokens[0].extent.start, tokens[-1].extent.end)
    #     stmt_file = stmt_extent[0].file.name
    #     file_slice = slice(stmt_extent[0].line, stmt_extent[-1].line)
    #     with open(stmt_file, "r") as f:
    #         stmt_text = f.readlines()[file_slice]

    #     ids = []
    #     cur_id = []
    #     for t in tokens:
    #         match t.kind:
    #             case tk.IDENTIFIER:
    #                 cur_id.append(t)
    #             case tk.PUNCTUATION:
    #                 match t.spelling:
    #                     case "::" | "->" | ".":
    #                         if cur_id:
    #                             cur_id.append(t)
    #                     case _:
    #                         if cur_id:
    #                             ids.append(cur_id)
    #                             cur_id = []
    #             case _:
    #                 if cur_id:
    #                     ids.append(cur_id)
    #                     cur_id = []

    #     return ids, stmt_text

    # def consume_lambda_entry(ts: List[ci.Token]):
    #     if not ts or ts[0].spelling != "[":
    #         return ts, None

    #     brack_level = 0
    #     for i, t in enumerate(ts):
    #         match t.spelling:
    #             case '[':
    #                 brack_level += 1
    #             case ']':
    #                 brack_level -= 1
    #                 if brack_level == 0:
    #                     return ts[i+1:], list(ts[1:i-1])

    #     return ts, None

    # def consume_braced_block(ts: List[ci.Token]):
    #     if not ts or ts[0].spelling != "{":
    #         return ts, None

    #     brack_stack = []

    #     for i, t in enumerate(ts):
    #         match t.kind:
    #             case tk.PUNCTUATION:
    #                 match t.spelling:
    #                     case '(' | '[' | '{':
    #                         brack_stack.append(t)
    #                     case ')' | ']' | '}':
    #                         if brack_stack[-1].spelling == BRACK_MAP[t.spelling]:
    #                             brack_stack.pop()
    #                             if len(brack_stack) == 0:
    #                                 return ts[i+1:], list(ts[:i])
    #                         else:
    #                             raise ValueError(
    #                                 f"Invalid brackets: {pt(brack_stack)}, {t.spelling}")
    #     return ts, None

    # def consume_lambda_def(ts: List[ci.Token]):
    #     ts, entry = consume_lambda_entry(ts)
    #     ts, args = consume_args(ts)
    #     ts, body = consume_braced_block(ts)
    #     return ts, body

    # def consume_callback_symbol(ts: List[ci.Token]):
    #     lambda_body = None
    #     if ts and ts[0].spelling == "std":
    #         ts, identifier, gen, args = consume_function_call(ts)
    #         if not args:
    #             raise ValueError("Empty arg list")
    #         if args[0][0].spelling != "&":
    #             raise NotImplementedError(pt(args))
    #         callback_sym = args[0][1:]
    #     elif ts and ts[0].spelling == "[":
    #         ts, lambda_body = consume_lambda_def(ts)
    #         callback_sym = None
    #     else:
    #         print(
    #             f"Error: {pt(ts[:30])} is a callback symbol of unknown structure")
    #         callback_sym = None

    #     return callback_sym, lambda_body

    # def get_mappings(tu: ci.TranslationUnit):
    #     cb_sym_to_identifiers_map = {}

    #     ts_all = list(tu.cursor.get_tokens())
    #     for i, t in enumerate(ts_all):
    #         if t.kind == ci.TokenKind.IDENTIFIER and t.spelling == "create_subscription":
    #             ts2, identifier, gen, args = consume_function_call(ts_all[i:])
    #             cb = args[2]
    #             cb_sym, body = consume_callback_symbol(cb)
    #             cb_sym_key = pt(cb_sym) if cb_sym is not None else pt(cb)
    #             print(cb_sym_key)
    #             if body is None:
    #                 body = find_body(tu.cursor, cb_sym)
    #             if body is not None:
    #                 #print("    ",pt(body))
    #                 identifiers, text_lines = get_identifiers_with_lines(body)
    #                 for l in text_lines:
    #                     print(l.rstrip())

    #                 if cb_sym_key not in cb_sym_to_identifiers_map:
    #                     cb_sym_to_identifiers_map[cb_sym_key] = {'identifiers': list(
    #                         map(pt, identifiers)), 'body_lines': text_lines}
    #                 else:
    #                     raise ValueError(
    #                         f"Multiple create_subscription with same handler: {cb_sym_key}")
    #             else:
    #                 cb_sym_to_identifiers_map[cb_sym_key] = {
    #                     'error': "No function body found"}
    #                 #raise ValueError(f"No function body found for {cb_sym_key}")
    #     return cb_sym_to_identifiers_map
