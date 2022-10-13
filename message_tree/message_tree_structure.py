from collections import namedtuple


E2EBreakdownItem = namedtuple("E2EBreakdownItem", ("type", "duration", "location"))
DepTree = namedtuple("DepTree", ("head", "deps"))


def depth(tree: DepTree):
    return 1 + max(map(depth, tree.deps), default=0)


def size(tree: DepTree):
    return 1 + sum(map(size, tree.deps))


def fanout(tree: DepTree):
    if not tree.deps:
        return 1

    return sum(map(fanout, tree.deps))
