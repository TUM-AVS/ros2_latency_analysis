import hashlib
import json
import os
import pickle
from typing import List

from IPython import get_ipython
from IPython.core.magic import (register_cell_magic, needs_local_scope)


@register_cell_magic
@needs_local_scope
def skip_if_false(line, cell, local_ns=None):
    condition_var = eval(line, None, local_ns)
    if condition_var:
        get_ipython().run_cell(cell)
        return None
    return f"Skipped (evaluated {line} to False)"


def left_abbreviate(string, limit=120):
    return string if len(string) <= limit else f"...{string[:limit - 3]}"


def stable_hash(obj):
    return hashlib.md5(json.dumps(obj).encode("utf-8")).hexdigest()[:10]


def parse_as(type, string):
    if any(issubclass(type, type2) for type2 in (str, bool, float, int)):
        return type(string)
    if issubclass(type, list) or issubclass(type, dict) or issubclass(type, set):
        val = json.loads(string)
        return type(val)
    raise ValueError(f"Unknown type {type.__name__}")


def cached(name, function, file_deps: List[str]):
    if not os.path.isdir("cache"):
        os.makedirs("cache", exist_ok=True)

    dep_time = 0.0
    for file in file_deps:
        # Get modified time of the current dependency
        m_time = os.path.getmtime(file) if os.path.exists(file) else 0.

        # Update dependency time to be the newest modified time of any dependency
        if m_time > dep_time:
            dep_time = m_time

        # Check directories recursively to get the newest modified time
        for root, dirs, files in os.walk(file):
            for f in files + dirs:
                filename = os.path.join(root, f)
                m_time = os.path.getmtime(filename)

                if m_time > dep_time:
                    dep_time = m_time

    deps_hash = stable_hash(sorted(file_deps))
    pkl_filename = f"cache/{name}_{deps_hash}.pkl"

    pkl_time = os.path.getmtime(pkl_filename) if os.path.exists(pkl_filename) else 0.

    if pkl_time > dep_time:
        with open(pkl_filename, "rb") as f:
            print(f"[CACHE] Found up-to-date cache entry ({pkl_filename}) for {name}, loading.")
            return pickle.load(f)

    if os.path.exists(pkl_filename):
        print(f"[CACHE] Data dependencies for {name} changed, deleting cached version.")
        os.remove(pkl_filename)

    print(f"[CACHE] Creating cache entry for {name} (in {pkl_filename}).")
    obj = function()

    with open(pkl_filename, "wb") as f:
        pickle.dump(obj, f)

    return obj
