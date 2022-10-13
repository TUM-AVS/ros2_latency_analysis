from collections import namedtuple


E2EBreakdownItem = namedtuple("E2EBreakdownItem", ("type", "duration", "location"))
DepTree = namedtuple("DepTree", ("head", "deps"))


def depth(tree: DepTree, lvl=0):
    if lvl > 1000:
        return 0
    return 1 + max(map(lambda d: depth(d, lvl + 1), tree.deps), default=0)


def size(tree: DepTree, lvl=0):
    if lvl > 1000:
        return 0
    return 1 + sum(map(lambda d: size(d, lvl + 1), tree.deps))


def fanout(tree: DepTree):
    if not tree.deps:
        return 1

    return sum(map(fanout, tree.deps))
