from dataclasses import dataclass
from typing import List

from tracing_interop.tr_types import TrPublishInstance, TrCallbackInstance


@dataclass
class DepTree:
    head: TrCallbackInstance | TrPublishInstance
    deps: List['DepTree']

    def depth(self):
        return 1 + max(map(DepTree.depth, self.deps), default=0)

    def size(self):
        return 1 + sum(map(DepTree.size, self.deps))

    def fanout(self):
        if not self.deps:
            return 1

        return sum(map(DepTree.fanout, self.deps))

    def e2e_lat(self):
        return self.head.timestamp - self.critical_path()[-1].timestamp

    def critical_path(self):
        if not self.deps:
            return [self.head]

        return [self.head, *min(map(DepTree.critical_path, self.deps), key=lambda ls: ls[-1].timestamp)]
