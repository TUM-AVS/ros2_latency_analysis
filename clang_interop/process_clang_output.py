import functools
import json
import os
import pickle
from typing import Iterable

import numpy as np
import pandas as pd

from clang_interop.cl_types import ClNode, ClField, ClTimer, ClMethod, ClPublisher, ClSubscription, ClMemberRef, ClContext, \
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

    return set(ret_list)


def dictify(elems, key='id'):
    return {getattr(e, key): e for e in elems}


def definitions_from_json(cb_dict, tu):
    nodes = []
    pubs = []
    subs = []
    timers = []
    accesses = []
    fields = []
    methods = []

    if "nodes" in cb_dict:
        for node in cb_dict["nodes"]:
            nodes.append(ClNode(node, tu))
            for field in node["fields"]:
                fields.append(ClField(field, tu))
            for method in node["methods"]:
                methods.append(ClMethod(method, tu))

    if "publishers" in cb_dict:
        for publisher in cb_dict["publishers"]:
            pubs.append(ClPublisher(publisher, tu))

    if "subscriptions" in cb_dict:
        for subscription in cb_dict["subscriptions"]:
            subs.append(ClSubscription(subscription, tu))
            if "callback" in subscription:
                methods.append(ClMethod(subscription["callback"], tu))

    if "timers" in cb_dict:
        for timer in cb_dict["timers"]:
            timers.append(ClTimer(timer, tu))
            if "callback" in timer:
                methods.append(ClMethod(timer["callback"], tu))

    if "accesses" in cb_dict:
        for access_type in cb_dict["accesses"]:
            for access in cb_dict["accesses"][access_type]:
                accesses.append(ClMemberRef(access, tu))
                if "method" in access["context"]:
                    methods.append(ClMethod(access["context"]["method"], tu))

    nodes = dedup(nodes)
    pubs = dedup(pubs)
    subs = dedup(subs)
    timers = dedup(timers)
    fields = dedup(fields)
    methods = dedup(methods)

    return nodes, pubs, subs, timers, fields, methods, accesses


def process_clang_output(directory=IN_DIR):
    all_tus = set()
    all_nodes = set()
    all_pubs = set()
    all_subs = set()
    all_timers = set()
    all_fields = set()
    all_methods = set()
    all_accesses = []
    all_deps = {}
    all_publications = {}

    for filename in os.listdir(IN_DIR):
        source_filename = SRC_FILE_NAME(filename)
        print(f"Processing {source_filename}")

        with open(os.path.join(IN_DIR, filename), "r") as f:
            cb_dict = json.load(f)
            if cb_dict is None:
                print(f"  [WARN ] Empty tool output detected in {filename}")
                continue

            tu = ClTranslationUnit(source_filename)
            all_tus.add(tu)

            nodes, pubs, subs, timers, fields, methods, accesses = definitions_from_json(cb_dict, tu)
            deps, publications = find_data_deps(accesses)

            all_nodes.update(nodes)
            all_pubs.update(pubs)
            all_subs.update(subs)
            all_timers.update(timers)
            all_fields.update(fields)
            all_methods.update(methods)
            all_accesses += accesses
            all_deps.update(deps)
            all_publications.update(publications)

    clang_context = ClContext(all_tus, all_nodes, all_pubs, all_subs, all_timers, all_fields, all_methods, all_accesses,
                              all_deps, all_publications)

    return clang_context


if __name__ == "__main__":
    clang_context = process_clang_output()

    with open(OUT_NAME, "wb") as f:
        pickle.dump(clang_context, f)

    print("Done.")
