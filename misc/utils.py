import ast
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
    """
    Jupyter cell magic to skip a cell based on if a global variable evaluates to `False`.
    Outputs a message that the cell has been skipped due to the given variable, or,
    if not skipped, just runs the cell as per normal.
    Usage: `%%skip_if_false MY_GLOBAL_VARIABLE` at the beginning of a cell.

    """
    condition_var = eval(line, None, local_ns)
    if condition_var:
        get_ipython().run_cell(cell)
        return None
    return f"Skipped (evaluated {line} to False)"


def stable_hash(obj):
    """
    Stable hashing of objects like lists etc.
    This is done by building the MD5 hash on the JSON representation of `obj`.
    :param obj: The object to hash
    :return: The hash as a string (10 characters)
    """
    return hashlib.md5(json.dumps(obj).encode("utf-8")).hexdigest()[:10]


def parse_as(target_type, string):
    """
    Parse `string` as type `target_type` on a best-effort basis.
    First `literal_eval`s `string` and then calls `target_type(...)` on the result.
    :param target_type: The desired type or function
    :param string: The input string to be parsed
    :return: The parsed object
    """
    obj = ast.literal_eval(string)
    return target_type(obj)


def cached(name, function, file_deps: List[str], disable_cache=False):
    """
    Caches the result of `function()` in `cache/name_XYZ.pkl`.
    When `file_deps` change (one or more of the deps have been modified or the list changes),
    the cache is invalidated and `function()` is evaluated again.
    In both the cached and non-cached case, the result of `function()` is returned.

    :param name: Cache name (only use filesystem-friendly characters)
    :param function: The function of which the result shall be cached. Will be called without arguments.
    :param file_deps: A list of file/folder names that can invalidate the cache. Folders are checked recursively.
    :param disable_cache: Disable caching and always run `function()` if `True`. Otherwise enable cache.
    :return: The result of `function()`, wither from the cache or from calling it.
    """

    if disable_cache:
        print(f"[CACHE] Cache disabled for {name}.")
        return function()

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
