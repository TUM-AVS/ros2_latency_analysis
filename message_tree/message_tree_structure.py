from collections import namedtuple


# Representation of the smallest unit in E2E calculations (e.g. a DDS time, a calculation time, an idle time)
E2EBreakdownItem = namedtuple("E2EBreakdownItem", ("type", "duration", "location"))
# A recursive tree structure used to represent dependencies
DepTree = namedtuple("DepTree", ("head", "deps"))


def depth(tree: DepTree, lvl=0):
    """
    The depth of `tree` (the length of the longest path from root to any leaf).
    Capped at 1000.
    :param tree: The tree
    :param lvl: Internal, leave at 0; Used for recursion threshold
    :return: The depth
    """
    if lvl > 1000:
        return 0
    return 1 + max(map(lambda d: depth(d, lvl + 1), tree.deps), default=0)


def size(tree: DepTree, lvl=0):
    """
    The number of nodes (including root, inner and leaf nodes) of `tree`
    :param tree: The tree
    :param lvl: Internal, leave at 0; Used for recursion threshold for trees over 1000 entries deep.
    :return: The number of nodes in `tree`
    """
    if lvl > 1000:
        return 0
    return 1 + sum(map(lambda d: size(d, lvl + 1), tree.deps))


def fanout(tree: DepTree):
    """
    The number of leaves of `tree`.
    :param tree: The tree
    :return: The number of leaves
    """
    if not tree.deps:
        return 1

    return sum(map(fanout, tree.deps))
