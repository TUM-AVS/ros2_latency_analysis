import functools
import json
import os
import pickle
import re
from typing import Tuple, Iterable

import numpy as np
import pandas as pd
import termcolor

from clang_interop.types import ClNode, ClField, ClTimer, ClMethod, ClPublisher, ClSubscription, ClMemberRef, ClContext, \
    ClTranslationUnit

IN_DIR = "/home/max/Projects/ma-ros2-internal-dependency-analyzer/output"
SRC_DIR = "/home/max/Projects/autoware/src"

OUT_NAME = "clang_objects.pkl"


def SRC_FILE_NAME(in_file_name: str):
    return os.path.join(SRC_DIR, in_file_name.replace("-", "/").replace(".json", ".cpp"))


ignored_idfs = set()


class SetEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, set):
            return list(o)
        match o:
            case set():
                return list(o)
            case list() | dict() | int() | float() | str():
                return json.JSONEncoder.default(self, o)
            case np.int64:
                return json.JSONEncoder.default(self, int(o))

        return json.JSONEncoder.default(self, o)


def fuse_fields(f1, f2):
    if f1 is None:
        return f2

    if f2 is None:
        return f1

    if f1 == f2:
        return f1

    raise ValueError(f"Inconsistent fields {f1=} and {f2=} cannot be fused")


def fuse_objects(o1, o2):
    field_names = o1.__dataclass_fields__.keys()
    for f in field_names:
        setattr(o1, f, fuse_fields(getattr(o1, f), getattr(o2, f)))
    return o1


def find_data_deps(accesses: Iterable[ClMemberRef]):
    writes = set()
    reads = set()
    publications = {}

    for member_ref in accesses:
        member_id = member_ref.member_chain[0] if member_ref.member_chain else None
        if member_id is None:
            print(f"[WARN ] MemberRef without any members in chain @ {member_ref.source_range}")
            continue

        dep_tuple = (member_ref.method_id, member_id)

        match member_ref.type:
            case "write":
                writes.add(dep_tuple)
            case "read":
                reads.add(dep_tuple)
            case "call" | "arg":
                writes.add(dep_tuple)
                reads.add(dep_tuple)
            case "pub":
                if member_ref.method_id not in publications:
                    publications[member_ref.method_id] = set()
                publications[member_ref.method_id].add(member_id)

    reads = pd.DataFrame.from_records(list(reads), columns=['method_id', 'member_id'])
    writes = pd.DataFrame.from_records(list(writes), columns=['method_id', 'member_id'])

    deps = {}

    for reading_method in reads["method_id"].unique().tolist():
        deps[reading_method] = set()

        read_members = reads[reads['method_id'] == reading_method]["member_id"].unique().tolist()

        for read_member in read_members:
            writing_methods = writes[writes['member_id'] == read_member]['method_id'].unique().tolist()
            deps[reading_method].update(writing_methods)

        deps[reading_method].discard(reading_method)  # Remove reflexive dependencies

    return deps, publications


def dedup(elems):
    hash_map = {}

    for e in elems:
        if e.__hash__() not in hash_map:
            hash_map[e.__hash__()] = []
        hash_map[e.__hash__()].append(e)

    ret_list = []
    for hash, elems in hash_map.items():
        if len(elems) == 1:
            ret_list += elems
            continue

        elem = functools.reduce(fuse_objects, elems[1:], elems[0])
        ret_list.append(elem)
        print(f"Fused {len(elems)} {type(elem)}s")

    return ret_list


def dictify(elems, key='id'):
    return {getattr(e, key): e for e in elems}


def definitions_from_json(cb_dict):
    nodes = []
    pubs = []
    subs = []
    timers = []
    accesses = []
    fields = []
    methods = []

    if "nodes" in cb_dict:
        for node in cb_dict["nodes"]:
            nodes.append(ClNode(node))
            for field in node["fields"]:
                fields.append(ClField(field))
            for method in node["methods"]:
                methods.append(ClMethod(method))

    if "publishers" in cb_dict:
        for publisher in cb_dict["publishers"]:
            pubs.append(ClPublisher(publisher))

    if "subscriptions" in cb_dict:
        for subscription in cb_dict["subscriptions"]:
            subs.append(ClSubscription(subscription))

    if "timers" in cb_dict:
        for timer in cb_dict["timers"]:
            timers.append(ClTimer(timer))

    if "accesses" in cb_dict:
        for access_type in cb_dict["accesses"]:
            for access in cb_dict["accesses"][access_type]:
                accesses.append(ClMemberRef(access))

    nodes = dictify(dedup(nodes))
    pubs = dictify(dedup(pubs), key='member_id')
    subs = dictify(dedup(subs), key='callback_id')
    timers = dictify(dedup(timers), key='callback_id')
    fields = dictify(dedup(fields))
    methods = dictify(dedup(methods))

    return nodes, pubs, subs, timers, fields, methods, accesses


def highlight(substr: str, text: str):
    regex = r"(?<=\W)({substr})(?=\W)|^({substr})$"
    return re.sub(regex.format(substr=substr), termcolor.colored(r"\1\2", 'magenta', attrs=['bold']), text)


def prompt_user(file: str, cb: str, idf: str, text: str) -> Tuple[str, bool, bool]:
    print('\n' * 5)
    print(f"{file.rstrip('.cpp').rstrip('.hpp')}\n->{cb}:")
    print(highlight(idf.split('::')[-1], text))
    answer = input(f"{highlight(idf, idf)}\n"
                   f"write (w), read (r), both (rw), ignore future (i) exit and save (q), undo (z), skip (Enter): ")
    if answer not in ["", "r", "w", "rw", "q", "z", "i"]:
        print(f"Invalid answer '{answer}', try again.")
        answer = prompt_user(file, cb, idf, text)

    if answer == 'i':
        ignored_idfs.add(idf)
    elif any(x in answer for x in ['r', 'w']):
        ignored_idfs.discard(idf)

    return answer, answer == "q", answer == "z"


def main(cbs):
    open_files = {}
    cb_rw_dict = {}

    jobs = []

    for cb_id, cb_dict in cbs.items():
        cb_rw_dict[cb_dict['qualified_name']] = {'reads': set(), 'writes': set()}
        for ref_dict in cb_dict['member_refs']:
            if ref_dict['file'] not in open_files:
                with open(ref_dict['file'], 'r') as f:
                    open_files[ref_dict['file']] = f.readlines()

            ln = ref_dict['start_line'] - 1
            text = open_files[ref_dict['file']]
            line = termcolor.colored(text[ln], None, "on_cyan")
            lines = [*text[ln - 3:ln], line, *text[ln + 1:ln + 4]]
            text = ''.join(lines)
            jobs.append((ref_dict['file'], cb_dict['qualified_name'], ref_dict['qualified_name'], text))

    i = 0
    do_undo = False
    while i < len(jobs):
        file, cb, idf, text = jobs[i]

        if do_undo:
            ignored_idfs.discard(idf)
            cb_rw_dict[cb]['reads'].discard(idf)
            cb_rw_dict[cb]['writes'].discard(idf)
            do_undo = False

        if idf in ignored_idfs:
            print("Ignoring", idf)
            i += 1
            continue

        if idf in cb_rw_dict[cb]['reads'] and idf in cb_rw_dict[cb]['writes']:
            print(f"{idf} is already written to and read from in {cb}, skipping.")
            i += 1
            continue

        classification, answ_quit, answ_undo = prompt_user(file, cb, idf, text)

        if answ_quit:
            del cb_rw_dict[file][cb]
            break
        elif answ_undo:
            i -= 1
            do_undo = True
            continue

        if 'r' in classification:
            cb_rw_dict[cb]['reads'].add(idf)
        if 'w' in classification:
            cb_rw_dict[cb]['writes'].add(idf)
        if not any(x in classification for x in ['r', 'w']):
            print(f"Ignoring occurences of {idf} in cb.")

        i += 1

    with open("deps.json", "w") as f:
        json.dump(cb_rw_dict, f, cls=SetEncoder)

    print("Done.")


def process_clang_output(directory=IN_DIR):
    clang_context = ClContext()

    for filename in os.listdir(IN_DIR):
        source_filename = SRC_FILE_NAME(filename)
        print(f"Processing {source_filename}")
        with open(os.path.join(IN_DIR, filename), "r") as f:
            cb_dict = json.load(f)
            if cb_dict is None:
                print(f"  [WARN ] Empty tool output detected in {filename}")
                continue

            nodes, pubs, subs, timers, fields, methods, accesses = definitions_from_json(cb_dict)
            deps, publications = find_data_deps(accesses)

            tu = ClTranslationUnit(deps, publications, nodes, pubs, subs, timers, fields, methods, accesses)
            clang_context.translation_units[source_filename] = tu

    return clang_context


if __name__ == "__main__":
    clang_context = process_clang_output()

    with open(OUT_NAME, "wb") as f:
        pickle.dump(clang_context, f)

    print("Done.")
