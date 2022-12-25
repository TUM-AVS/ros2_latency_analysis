import re
from bisect import bisect_right
from collections import defaultdict
from functools import cache
from typing import List, Optional

from tqdm import tqdm

from latency_graph.latency_graph_structure import LatencyGraph
from matching.subscriptions import sanitize
from message_tree.message_tree_structure import DepTree, E2EBreakdownItem, depth, size
from tracing_interop.tr_types import TrCallbackInstance, TrPublishInstance, TrPublisher, TrCallbackObject, TrContext, \
    TrSubscriptionObject, TrTimer, TrNode, TrTopic


__dep_tree_cache = {}


def _repr(inst: TrCallbackInstance | TrPublishInstance):
    """
    If a string representation is found for `inst`, it is returned, else an empty string is returned.
    """
    match inst:
        case TrPublishInstance(publisher=pub):
            return pub.topic_name if pub else ""
        case TrCallbackInstance(callback_obj=cb_obj):
            cb_obj: TrCallbackObject
            return repr(sanitize(cb_obj.callback_symbol.symbol)) if cb_obj and cb_obj.callback_symbol else ""
    raise TypeError(f"Argument has to be callback or publish instance, is {type(inst).__name__}")


def get_dep_tree(inst: TrPublishInstance | TrCallbackInstance, lat_graph: LatencyGraph, tr: TrContext,
                 excluded_path_patterns, time_limit_s, exact_path: Optional[List[str]] = None):
    """
    Finds all (desired) dependencies of a publish/callback instance and returns a dependency tree.

    The dependencies can be filtered via `time_limit_s`, which is the maximum difference in start time between `inst`
    and any dependency in the tree.
    Another filter is `excluded_path_patterns`, which cuts off paths where one instance's string representation matches
    any of the given regex patterns.
    """


    start_time = inst.timestamp

    def __get_dep_tree(inst, is_dep_cb, visited=None, level=0):

        # If inst owner has been visited already, skip (loop prevention)
        if visited is not None and owner(inst) in visited:
            return None

        # If an exact path is given, the owner of `inst` at `level` has to be identical to the corresponding item
        # in `exact_path`. The level is non-positive (start at 0, descend one per recursion).
        # `exact_path` is therefore traversed from back to front.
        # If `level` is deeper than `exact_path` is long, the tree is also not of interest and thus discarded.
        if exact_path is not None and ((level - 1 < -len(exact_path)) or exact_path[level - 1] != owner(inst)):
            return None

        # If we want to retrieve the tree, look in the cache first
        cache_key = (inst, is_dep_cb)
        if cache_key in __dep_tree_cache:
            return __dep_tree_cache[cache_key]

        if visited is None:
            visited = tuple()

        if any(re.search(p, _repr(inst)) for p in excluded_path_patterns):
            return None

        if inst.timestamp - start_time > time_limit_s:
            return None

        children_are_dep_cbs = False

        match inst:
            case TrPublishInstance(publisher=pub):
                deps = [get_msg_dep_cb(inst, lat_graph)]
            case TrCallbackInstance() as cb_inst:
                cb_inst: TrCallbackInstance
                deps = [inst_get_dep_msg(cb_inst, tr)]
                if not is_dep_cb:
                    deps += inst_get_dep_insts(cb_inst, tr)
                    children_are_dep_cbs = True
            case _:
                print("[WARN] Expected inst to be of type TrPublishInstance or TrCallbackInstance, "
                      f"got {type(inst).__name__}")
                return None
        # print("Rec level", lvl)
        deps = list(filter(None, deps))
        # Copy visited set for each child because we don't want separate paths to interfere with each other
        deps = [__get_dep_tree(dep, children_are_dep_cbs, {*visited, owner(inst)}, level=level-1) for dep in deps]
        deps = list(filter(None, deps))

        # Create tree instance, cache and return it
        ret_tree = DepTree(inst, deps)
        __dep_tree_cache[cache_key] = ret_tree
        return ret_tree

    return __get_dep_tree(inst, False)


def build_dep_trees(end_topics, lat_graph, tr, excluded_path_patterns, time_limit_s, exact_path=None):
    """
    Builds the dependency trees for all messages published in any of `end_topics` and returns them as a list.
    """
    all_trees = []
    for end_topic in end_topics:
        end_topic: TrTopic
        print(f"====={end_topic.name}")

        pubs = end_topic.publishers
        for pub in pubs:
            msgs = list(pub.instances)
            for msg in tqdm(msgs, mininterval=5.0, desc="Processing output messages"):
                msg: TrPublishInstance
                tree = get_dep_tree(msg, lat_graph, tr, excluded_path_patterns, time_limit_s, exact_path=exact_path)
                all_trees.append(tree)
    return all_trees


def inst_get_dep_msg(inst: TrCallbackInstance, tr: TrContext):
    if inst.callback_object not in tr.callback_objects.by_callback_object:
        # print("Callback not found (2)")
        return None

    if not isinstance(inst.callback_obj.owner, TrSubscriptionObject):
        # print(f"Wrong type: {type(inst.callback_obj.owner)}")
        return None

    sub_obj: TrSubscriptionObject = inst.callback_obj.owner
    if sub_obj and sub_obj.subscription and sub_obj.subscription.topics:
        # print(f"Subscription has no topic")
        pubs = [p for t in sub_obj.subscription.topics for p in t.publishers]
    else:
        pubs = []

    def _pub_latest_msg_before(pub: TrPublisher, inst):
        i_latest_msg = bisect_right(pub.instances, inst.timestamp, key=lambda x: x.timestamp) - 1
        if i_latest_msg < 0 or i_latest_msg >= len(pub.instances):
            return None
        latest_msg = pub.instances[i_latest_msg]
        if latest_msg.timestamp >= inst.timestamp:
            return None

        return latest_msg

    msgs = [_pub_latest_msg_before(pub, inst) for pub in pubs]
    msgs = list(filter(None, msgs))
    msgs.sort(key=lambda i: i.timestamp, reverse=True)
    if msgs:
        msg = msgs[0]
        return msg

    return None


def inst_get_dep_insts(inst: TrCallbackInstance, tr: TrContext):
    if inst.callback_object not in tr.callback_objects.by_callback_object:
        # print("Callback not found")
        return []
    dep_cbs = get_cb_dep_cbs(inst.callback_obj, tr)

    def _cb_to_chronological_inst(cb: TrCallbackObject, inst):
        i_inst_latest = bisect_right(cb.callback_instances, inst.timestamp, key=lambda x: x.timestamp)

        for inst_before in cb.callback_instances[i_inst_latest::-1]:
            if inst_before.t_end < inst.timestamp:
                return inst_before

        return None

    insts = [_cb_to_chronological_inst(cb, inst) for cb in dep_cbs]
    insts = list(filter(None, insts))
    return insts


def get_cb_dep_cbs(cb: TrCallbackObject, tr: TrContext):
    match cb.owner:
        case TrSubscriptionObject() as sub_obj:
            sub_obj: TrSubscriptionObject
            owner = sub_obj.subscription.node
        case TrTimer() as tmr:
            tmr: TrTimer
            owner = tmr.node
        case _:
            raise RuntimeError(f"Encountered {cb.owner} as callback owner")

    owner: TrNode
    dep_sub_objs = {sub.subscription_object for sub in owner.subscriptions}
    dep_cbs = {tr.callback_objects.by_id.get(sub_obj.id) for sub_obj in dep_sub_objs if sub_obj is not None}
    dep_cbs |= {tr.callback_objects.by_id.get(tmr.id) for tmr in owner.timers}
    dep_cbs.discard(cb)
    dep_cbs.discard(None)

    return dep_cbs


def get_msg_dep_cb(msg: TrPublishInstance, lat_graph: LatencyGraph):
    """
    For a given message instance `msg`, find the publishing callback,
    as well as the message instances that callback depends on (transitively within its TrNode).
    """

    # Find CB instance that published msg
    # print(f"PUB {msg.publisher.node.path if msg.publisher.node is not None else '??'} ==> {msg.publisher.topic_name}")
    pub_cbs = lat_graph.pub_cbs.get(msg.publisher)
    if pub_cbs is None:
        # print("Publisher unknown to lat graph. Skipping.")
        return None

    # print(f"Found {len(pub_cbs)} pub cbs")
    cb_inst_candidates = []
    for cb in pub_cbs:
        # print(f"  > CB ({len(cb.callback_instances)} instances): {cb.callback_symbol.symbol if cb.callback_symbol else cb.id}")
        i_inst_after = bisect_right(cb.callback_instances, msg.timestamp, key=lambda x: x.timestamp)

        for inst in cb.callback_instances[:i_inst_after]:
            if msg.timestamp > inst.t_end:
                continue

            assert inst.t_start <= msg.timestamp <= inst.t_end

            cb_inst_candidates.append(inst)

    if len(cb_inst_candidates) > 1:
        # print("Found multiple possible callbacks")
        return None
    if not cb_inst_candidates:
        # print("Found no possible callbacks")
        return None

    dep_inst = cb_inst_candidates[0]
    return dep_inst


def e2e_paths_sorted_desc(tree: DepTree, input_topic_patterns):
    """
    Return all paths through `tree` that start with a callback instance publishing on a topic matching any of
    `input_topic_patterns`. The paths are sorted by length in a descending manner (element 0 is longest).
    """

    # TODO: Make this a Dijkstra/similar implementation instead. This is so slow it's funny.
    def _collect_all_paths(t: DepTree, lvl=0):
        """
        Returns a flat list of all paths that lead from t's root to a leaf.
        """
        if lvl > 30 or not t.deps:
            return [(t.head, )]

        # List of flat lists of paths
        deps_paths = [_collect_all_paths(d, lvl + 1) for d in t.deps]
        return [(*path, t.head) for dep_paths in deps_paths for path in dep_paths]

    def _trim_path(path):
        valid_input = False
        i = -1
        for i, inst in enumerate(path):
            match inst:
                case TrPublishInstance(publisher=pub):
                    pub: TrPublisher
                    if pub and any(re.search(p, pub.topic_name) for p in input_topic_patterns):
                        valid_input = True
                        break

        if not valid_input:
            return None

        if i == 0:
            # print(end=".")
            return path  # Return whole path, even if there is not publishing callback
        if not isinstance(path[i - 1], TrCallbackInstance):
            # print(end="#")
            return None

        return path[i - 1:]  # Return path from its publishing callback if it exists

    paths = _collect_all_paths(tree)
    paths = list(filter(None, map(_trim_path, paths)))
    paths.sort(key=lambda path: path[-1].timestamp - path[0].timestamp, reverse=True)
    return paths


def e2e_latency_breakdown(path: list):
    """
    Separates E2E latency into a sequence of dds, idle, and cpu times.
    This method expects a publish instance at the last position in `path`.

    The return format is a list of the form [("<type>", <time>), ("<type>", <time>), ...] with type bein gone of the
    three mentioned above.
    """
    ret_list: List[E2EBreakdownItem] = []

    cb_inst: TrCallbackInstance
    cb_inst_prev: TrCallbackInstance
    pub_inst: TrPublishInstance
    pub_inst_prev: TrPublishInstance

    last_inst = None
    for inst in path:
        match inst:
            case TrCallbackInstance() as cb_inst:
                match last_inst:
                    case TrCallbackInstance() as cb_inst_prev:
                        ret_list.append(E2EBreakdownItem("cpu", cb_inst_prev.duration,
                                                         (cb_inst_prev, cb_inst_prev)))
                        ret_list.append(E2EBreakdownItem("idle", cb_inst.t_start - cb_inst_prev.t_end,
                                                         (cb_inst_prev, cb_inst)))
                    case TrPublishInstance() as pub_inst_prev:
                        ret_list.append(E2EBreakdownItem("dds", cb_inst.t_start - pub_inst_prev.timestamp,
                                                         (pub_inst_prev, cb_inst)))
            case TrPublishInstance() as pub_inst:
                match last_inst:
                    case TrCallbackInstance() as cb_inst_prev:
                        ret_list.append(E2EBreakdownItem("cpu", pub_inst.timestamp - cb_inst_prev.t_start,
                                                         (cb_inst_prev, pub_inst)))
                    case TrPublishInstance():
                        raise TypeError(f"Found two publish instances in a row in an E2E path.")
        last_inst = inst

    if not isinstance(last_inst, TrPublishInstance):
        raise TypeError(f"Last instance in path is not a message but a {type(last_inst).__name__}")

    return ret_list


@cache
def owner(inst: TrCallbackInstance | TrPublishInstance):
    match inst:
        case TrCallbackInstance(callback_obj=cb_obj):
            cb_obj: TrCallbackObject
            if cb_obj and cb_obj.callback_symbol:
                sym = repr(sanitize(cb_obj.callback_symbol.symbol))
            else:
                sym = str(cb_obj.id)
            return sym
        case TrPublishInstance(publisher=pub):
            pub: TrPublisher
            topic = pub.topic_name
            return topic
        case _:
            raise ValueError()


def _repr_path(path: List[TrPublishInstance | TrCallbackInstance]):
    return " -> ".join(map(owner, path))


def aggregate_e2e_paths(paths: List[List[TrPublishInstance | TrCallbackInstance]]):
    path_cohorts = defaultdict(list)

    for path in tqdm(paths, mininterval=5.0, desc="Aggregating E2E path cohorts"):
        key = _repr_path(path)
        path_cohorts[key].append(path)

    return path_cohorts


def label_latency_item(item: E2EBreakdownItem):
    match item.type:
        case "cpu":
            return f"{_repr(item.location[0])}"
        case "idle":
            cb_inst: TrCallbackInstance = item.location[0]
            owner = cb_inst.callback_obj.owner
            match owner:
                case TrTimer() as tmr:
                    tmr: TrTimer
                    node_name = tmr.node.path
                case TrSubscriptionObject() as sub:
                    sub: TrSubscriptionObject
                    node_name = sub.subscription.node.path
                case _:
                    raise TypeError()
            return f"{node_name}"
        case "dds":
            msg_inst: TrPublishInstance = item.location[0]
            return f"{msg_inst.publisher.topic_name}"
        case _:
            return ValueError()
