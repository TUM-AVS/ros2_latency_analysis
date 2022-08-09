from dataclasses import dataclass
from itertools import combinations
from multiprocessing import Pool
from typing import Optional, Set, List, Iterable, Dict, Tuple

from tqdm.notebook import tqdm
from tqdm.contrib import concurrent

from matching.subscriptions import sanitize
from tracing_interop.tr_types import TrContext, TrCallbackObject, TrCallbackSymbol, TrNode, TrPublisher, TrSubscription, \
    TrTimer, TrPublishInstance, TrSubscriptionObject, TrTopic, TrCallbackInstance


TOPIC_FILTERS = ["/parameter_events", "/tf_static", "/robot_description", "diagnostics"]


def _map_cb_times(args):
    cb_id, inst_times, pub_timestamps = args
    pub_cb_overlaps = {i: set() for i in range(len(pub_timestamps))}

    inst_times.sort(key=lambda tup: tup[0])  # tup[0] is start time

    inst_iter = iter(inst_times)
    pub_iter = iter(enumerate(pub_timestamps))

    inst_start, inst_end = next(inst_iter, (None, None))
    i, t = next(pub_iter, (None, None))
    while inst_start is not None and i is not None:
        if inst_start <= t <= inst_end:
            pub_cb_overlaps[i].add(cb_id)

        if t <= inst_end:
            i, t = next(pub_iter, (None, None))
        else:
            inst_start, inst_end = next(inst_iter, (None, None))

    return pub_cb_overlaps


def _get_cb_owner_node(cb: TrCallbackObject) -> TrNode | None:
    match cb.owner:
        case TrTimer(nodes=nodes):
            owner_nodes = nodes
        case TrSubscriptionObject(subscription=sub):
            owner_nodes = [sub.node]
        case _:
            owner_nodes = []

    if len(owner_nodes) > 1:
        raise RuntimeError(f"CB has owners {', '.join(map(lambda n: n.path, owner_nodes))}")
    elif not owner_nodes:
        print("[WARN] CB has no owners")
        return None

    return owner_nodes[0]


def _hierarchize(lg_nodes: Iterable['LGHierarchyLevel']):
    base = LGHierarchyLevel(None, [], "", [])

    def _insert(parent, node, path):
        match path:
            case []:
                parent.children.append(node)
                node.parent = parent
            case [head, *tail]:
                next_node = next(iter(n for n in parent.children if n.name == head), None)
                if next_node is None:
                    next_node = LGHierarchyLevel(parent, [], head, [])
                    parent.children.append(next_node)
                _insert(next_node, node, tail)

    for node in lg_nodes:
        path = node.name.strip("/").split("/")
        node.name = path[-1]
        _insert(base, node, path[:-1])

    return base


def inst_runtime_interval(cb_inst: TrCallbackInstance):
    inst_t_min = cb_inst.timestamp.timestamp()
    inst_t_max = inst_t_min + cb_inst.duration.total_seconds()
    return inst_t_min, inst_t_max


def _get_publishing_cbs(cbs: Set[TrCallbackObject], pub: TrPublisher):
    """
    Counts number of publication instances that lie within one of the cb_intervals.
    """
    pub_timestamps = [inst.timestamp * 1e-9 for inst in pub.instances]

    # Algorithm: Two-pointer method
    # With both the pub_timestamps and cb_intervals sorted ascending,
    # we can cut down the O(m*n) comparisons to O(m+n).
    pub_timestamps.sort()

    cb_id_to_cb = {cb.id: cb for cb in cbs}
    _map_args = [(cb.id, [inst_runtime_interval(inst) for inst in cb.callback_instances], pub_timestamps) for cb in cbs]

    with Pool() as p:
        cb_wise_overlaps = p.map(_map_cb_times, _map_args)

    pub_cb_overlaps = {i: set() for i in range(len(pub_timestamps))}
    for overlap_dict in cb_wise_overlaps:
        for i, cb_ids in overlap_dict.items():
            cbs = [cb_id_to_cb[cb_id] for cb_id in cb_ids]
            pub_cb_overlaps[i].update(cbs)

    pub_cbs = set()
    cb_cb_overlaps = set()
    for i, i_cbs in pub_cb_overlaps.items():
        if not i_cbs:
            print(f"[WARN] Publication on {pub.topic_name} without corresponding callback!")
        elif len(i_cbs) == 1:
            pub_cbs.update(i_cbs)
        else:  # Multiple CBs in i_cbs
            cb_cb_overlaps.update(iter(combinations(i_cbs, 2)))

    for cb1, cb2 in cb_cb_overlaps:
        cb1_subset_of_cb2 = True
        cb2_subset_of_cb1 = True

        for i_cbs in pub_cb_overlaps.values():
            if cb1 in i_cbs and cb2 not in i_cbs:
                cb1_subset_of_cb2 = False
            if cb2 in i_cbs and cb1 not in i_cbs:
                cb2_subset_of_cb1 = False

        if cb1_subset_of_cb2 and cb2_subset_of_cb1:
            print(f"[WARN] Callbacks {cb1.id} and {cb2.id} always run in parallel")
        elif cb1_subset_of_cb2:
            pub_cbs.discard(cb1)
        elif cb2_subset_of_cb1:
            pub_cbs.discard(cb2)
        # else: discard none of them

    return pub_cbs


def _get_cb_topic_deps(nodes_to_cbs: Dict[TrNode, Set[TrCallbackObject]]):
    cbs_subbed_to_topic: Dict[TrTopic, Set[TrCallbackObject]] = {}

    # Find topics the callback EXPLICITLY depends on
    # - Timer callbacks: no EXPLICIT dependencies
    # - Subscription callbacks: CB depends on the subscribed topic. Possibly also has other IMPLICIT dependencies
    p = tqdm(desc="Processing CB subscriptions", total=sum(map(len, nodes_to_cbs.values())))
    for node, cbs in nodes_to_cbs.items():
        for cb in cbs:
            p.update()

            if type(cb.owner) == TrSubscriptionObject:
                dep_topics = [cb.owner.subscription.topic]
            elif type(cb.owner) == TrTimer:
                dep_topics = []
            elif cb.owner is None:
                continue
            else:
                raise RuntimeError(
                        f"Callback owners other than timers/subscriptions cannot be handled: {cb.owner}")

            for topic in dep_topics:
                if topic not in cbs_subbed_to_topic:
                    cbs_subbed_to_topic[topic] = set()
                cbs_subbed_to_topic[topic].add(cb)

    # Find topics the callback publishes to (HEURISTICALLY!)
    # For topics published to during the runtime of the callback's instances,
    # assume that they are published by the callback
    cbs_publishing_topic: Dict[TrTopic, Set[TrCallbackObject]] = {}
    p = tqdm(desc="Processing node publications", total=len(nodes_to_cbs))
    for node, cbs in nodes_to_cbs.items():
        p.update()
        if node is None:
            continue
        for pub in node.publishers:
            if any(f in pub.topic_name for f in TOPIC_FILTERS):
                continue
            pub_cbs = _get_publishing_cbs(cbs, pub)
            if pub.topic not in cbs_publishing_topic:
                cbs_publishing_topic[pub.topic] = set()

            cbs_publishing_topic[pub.topic].update(pub_cbs)

    return cbs_subbed_to_topic, cbs_publishing_topic


@dataclass
class LGCallback:
    name: str
    in_topics: List[TrTopic]
    out_topics: List[TrTopic]

    def id(self):
        return self.name


@dataclass
class LGTrCallback(LGCallback):
    cb: TrCallbackObject
    sym: TrCallbackSymbol | None
    node: TrNode | None

    def id(self):
        return str(self.cb.id)


@dataclass
class LGHierarchyLevel:
    parent: Optional['LGHierarchyLevel']
    children: List['LGHierarchyLevel']
    name: str
    callbacks: List[LGCallback]

    @property
    def full_name(self):
        if self.parent is None:
            return f"{self.name}"

        return f"{self.parent.full_name}/{self.name}"


@dataclass
class LGEdge:
    start: LGCallback
    end: LGCallback


@dataclass
class LatencyGraph:
    top_node: LGHierarchyLevel
    edges: List[LGEdge]

    def __init__(self, tr: TrContext):
        ##################################################
        # Annotate nodes with their callbacks
        ##################################################

        # Note that nodes can also be None!
        nodes_to_cbs = {}
        p = tqdm(desc="Finding CB nodes", total=len(tr.callback_objects))
        for cb in tr.callback_objects.values():
            p.update()
            node = _get_cb_owner_node(cb)

            if node not in nodes_to_cbs:
                nodes_to_cbs[node] = set()
            nodes_to_cbs[node].add(cb)

        ##################################################
        # Find in/out topics for each callback
        ##################################################

        cbs_subbed_to_topic, cbs_publishing_topic = _get_cb_topic_deps(nodes_to_cbs)

        ##################################################
        # Map topics to their messages
        ##################################################

        topics_to_messages = {}
        p = tqdm(desc="Mapping messages to topics", total=len(tr.publish_instances))
        for pub_inst in tr.publish_instances:
            p.update()
            try:
                topic = pub_inst.publisher.topic
            except KeyError:
                continue

            if topic not in topics_to_messages:
                topics_to_messages[topic] = []
            topics_to_messages[topic].append(pub_inst)

        ##################################################
        # Define nodes and edges on lowest level
        ##################################################

        input = LGCallback("INPUT", [], [topic for topic in tr.topics.values() if not topic.publishers])
        output = LGCallback("OUTPUT", [topic for topic in tr.topics.values() if not topic.subscriptions], [])

        in_node = LGHierarchyLevel(None, [], "INPUT", [input])
        out_node = LGHierarchyLevel(None, [], "OUTPUT", [output])

        lg_nodes = [in_node, out_node]

        tr_to_lg_cb = {}

        p = tqdm("Building graph nodes", total=sum(map(len, nodes_to_cbs.values())))
        for node, cbs in nodes_to_cbs.items():
            node_callbacks = []

            for cb in cbs:
                p.update()
                try:
                    sym = cb.callback_symbol
                    pretty_sym = sanitize(sym.symbol)
                except KeyError:
                    sym = None
                    pretty_sym = cb.id
                in_topics = [topic for topic, cbs in cbs_subbed_to_topic.items() if cb in cbs]
                out_topics = [topic for topic, cbs in cbs_publishing_topic.items() if cb in cbs]
                lg_cb = LGTrCallback(pretty_sym, in_topics, out_topics, cb, sym, node)
                node_callbacks.append(lg_cb)
                tr_to_lg_cb[cb] = lg_cb

            lg_node = LGHierarchyLevel(None, [], node.path if node else "[NONE]", node_callbacks)
            lg_nodes.append(lg_node)

        edges = []
        p = tqdm("Building graph edges", total=len(tr.topics))
        for topic in tr.topics.values():
            p.update()
            sub_cbs = cbs_subbed_to_topic[topic] if topic in cbs_subbed_to_topic else []
            pub_cbs = cbs_publishing_topic[topic] if topic in cbs_publishing_topic else []

            for sub_cb in sub_cbs:
                for pub_cb in pub_cbs:
                    lg_edge = LGEdge(tr_to_lg_cb[pub_cb], tr_to_lg_cb[sub_cb])
                    edges.append(lg_edge)

        self.edges = edges

        ##################################################
        # Nodes into hierarchy levels
        ##################################################

        self.top_node = _hierarchize(lg_nodes)

    def to_gv(self):
        pass
