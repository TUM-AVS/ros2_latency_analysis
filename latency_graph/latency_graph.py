from bisect import bisect_left, bisect_right
from collections import defaultdict
from dataclasses import dataclass
from itertools import combinations
from multiprocessing import Pool
from typing import Optional, Set, List, Iterable, Dict, Tuple
from functools import cache

import numpy as np
from tqdm import tqdm
from tqdm.contrib import concurrent

from matching.subscriptions import sanitize
from tracing_interop.tr_types import TrContext, TrCallbackObject, TrCallbackSymbol, TrNode, TrPublisher, TrSubscription, \
    TrTimer, TrPublishInstance, TrSubscriptionObject, TrTopic, TrCallbackInstance, Timestamp

TOPIC_FILTERS = ["/parameter_events", "/tf_static", "/robot_description", "diagnostics", "/rosout"]


@cache
def _get_cb_owner_node(cb: TrCallbackObject) -> TrNode | None:
    match cb.owner:
        case TrTimer(node=node):
            return node
        case TrSubscriptionObject(subscription=sub):
            return sub.node
        case _:
            return None


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


def _get_publishing_cbs(cbs: Set[TrCallbackObject], pub: TrPublisher):
    """
    Counts number of publication instances that lie within one of the cb_intervals.
    """
    pub_insts = pub.instances
    pub_cb_overlaps = {i: set() for i in range(len(pub_insts))}

    for cb in cbs:
        cb_intervals = map(lambda inst: (inst.t_start, inst.t_end), cb.callback_instances)
        for t_start, t_end in cb_intervals:
            i_overlap_begin = bisect_left(pub_insts, t_start, key=lambda x: x.timestamp)
            i_overlap_end = bisect_right(pub_insts, t_end, key=lambda x: x.timestamp)
            for i in range(i_overlap_begin, i_overlap_end):
                pub_cb_overlaps[i].add(cb)

    pub_cbs = set()
    cb_cb_overlaps = set()
    cb_less_pubs = defaultdict(lambda: 0)
    for i, i_cbs in pub_cb_overlaps.items():
        if not i_cbs:
            cb_less_pubs[pub.topic_name] += 1
        elif len(i_cbs) == 1:
            pub_cbs.update(i_cbs)
        else:  # Multiple CBs in i_cbs
            cb_cb_overlaps.update(iter(combinations(i_cbs, 2)))

    if cb_less_pubs:
        print(f"[ENTKÄFERN] There are {len(cb_less_pubs)} topics which have publications from untracked callbacks:")
    for topic_name, count in cb_less_pubs.items():
        print(f"--{pub.topic_name:.<120s}: {count:>20d} ownerless publications")

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
                print(f" [WARN] Callback owners other than timers/subscriptions cannot be handled: {cb.owner}")
                continue

            for topic in dep_topics:
                if topic not in cbs_subbed_to_topic:
                    cbs_subbed_to_topic[topic] = set()
                cbs_subbed_to_topic[topic].add(cb)

    # Find topics the callback publishes to (HEURISTICALLY!)
    # For topics published to during the runtime of the callback's instances,
    # assume that they are published by the callback
    cbs_publishing_topic: Dict[TrTopic, Set[TrCallbackObject]] = {}
    cb_publishers: Dict[TrCallbackObject, Set[TrPublisher]] = {}
    for node, cbs in tqdm(nodes_to_cbs.items(), desc="Processing node publications"):
        if node is None:
            continue
        for pub in node.publishers:
            if any(f in pub.topic_name for f in TOPIC_FILTERS):
                continue
            pub_cbs = _get_publishing_cbs(cbs, pub)
            for cb in pub_cbs:
                if cb not in cb_publishers:
                    cb_publishers[cb] = set()
                cb_publishers[cb].add(pub)
            if pub.topic not in cbs_publishing_topic:
                cbs_publishing_topic[pub.topic] = set()

            cbs_publishing_topic[pub.topic].update(pub_cbs)

    return cbs_subbed_to_topic, cbs_publishing_topic, cb_publishers


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
    topic: TrTopic


@dataclass
class LatencyGraph:
    top_node: LGHierarchyLevel
    edges: List[LGEdge]

    cb_pubs: Dict[TrCallbackObject, Set[TrPublisher]]
    pub_cbs: Dict[TrPublisher, Set[TrCallbackObject]]

    def __init__(self, tr: TrContext):
        ##################################################
        # Annotate nodes with their callbacks
        ##################################################

        # Note that nodes can also be None!
        nodes_to_cbs = {}
        ownerless_cbs = 0
        for cb in tqdm(tr.callback_objects, desc="Finding CB nodes"):
            node = _get_cb_owner_node(cb)
            if node is None:
                ownerless_cbs += 1
                continue

            if node not in nodes_to_cbs:
                nodes_to_cbs[node] = set()
            nodes_to_cbs[node].add(cb)

        print(f"[ENTKÄFERN] {ownerless_cbs} callbacks have no owner, filtering them out.")

        ##################################################
        # Find in/out topics for each callback
        ##################################################

        cbs_subbed_to_topic, cbs_publishing_topic, cb_pubs = _get_cb_topic_deps(nodes_to_cbs)
        pub_cbs = {}
        for cb, pubs in cb_pubs.items():
            for pub in pubs:
                if pub not in pub_cbs:
                    pub_cbs[pub] = set()
                pub_cbs[pub].add(cb)

        self.cb_pubs = cb_pubs
        self.pub_cbs = pub_cbs

        ##################################################
        # Define nodes and edges on lowest level
        ##################################################

        input = LGCallback("INPUT", [], [topic for topic in tr.topics if not topic.publishers])
        output = LGCallback("OUTPUT", [topic for topic in tr.topics if not topic.subscriptions], [])

        in_node = LGHierarchyLevel(None, [], "INPUT", [input])
        out_node = LGHierarchyLevel(None, [], "OUTPUT", [output])

        lg_nodes = [in_node, out_node]

        tr_to_lg_cb = {}

        p = tqdm(desc="Building graph nodes", total=sum(map(len, nodes_to_cbs.values())))
        for node, cbs in nodes_to_cbs.items():
            node_callbacks = []

            for cb in cbs:
                p.update()

                sym = cb.callback_symbol
                if sym is not None:
                    pretty_sym = sanitize(sym.symbol)
                else:
                    pretty_sym = cb.id
                in_topics = [topic for topic, cbs in cbs_subbed_to_topic.items() if cb in cbs]
                out_topics = [topic for topic, cbs in cbs_publishing_topic.items() if cb in cbs]
                lg_cb = LGTrCallback(pretty_sym, in_topics, out_topics, cb, sym, node)
                node_callbacks.append(lg_cb)
                tr_to_lg_cb[cb] = lg_cb

            lg_node = LGHierarchyLevel(None, [], node.path if node else "[NONE]", node_callbacks)
            lg_nodes.append(lg_node)

        edges = []
        for topic in tqdm(tr.topics, desc="Building graph edges"):
            sub_cbs = cbs_subbed_to_topic[topic] if topic in cbs_subbed_to_topic else []
            pub_cbs = cbs_publishing_topic[topic] if topic in cbs_publishing_topic else []

            for sub_cb in sub_cbs:
                for pub_cb in pub_cbs:
                    lg_edge = LGEdge(tr_to_lg_cb[pub_cb], tr_to_lg_cb[sub_cb], topic)
                    edges.append(lg_edge)

        self.edges = edges

        ##################################################
        # Nodes into hierarchy levels
        ##################################################

        self.top_node = _hierarchize(lg_nodes)
