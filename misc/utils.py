import base64
import glob
import hashlib
import json
import math
import os
import pickle
import time
from typing import List


def left_abbreviate(string, limit=120):
    return string if len(string) <= limit else f"...{string[:limit - 3]}"


class ProgressPrinter:
    def __init__(self, verb, n) -> None:
        self.verb = verb
        self.n = n
        self.i = 0
        self.fmt_len = math.ceil(math.log10(n if n > 0 else 1))

    def step(self, msg):
        self.i += 1
        print(f"({self.i:>{self.fmt_len}d}/{self.n}) {self.verb} {left_abbreviate(msg):<120}", end="\r")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.i -= 1

        if exc_value:
            self.step("error.")
            print()
            print(exc_value)
            return

        self.step("done.")
        print()


def stable_hash(obj):
    return hashlib.md5(json.dumps(obj).encode("utf-8")).hexdigest()[:10]


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
