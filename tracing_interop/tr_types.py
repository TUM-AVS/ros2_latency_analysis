from uuid import UUID, uuid4
from collections import namedtuple, UserList, defaultdict
from dataclasses import dataclass, field
from functools import cached_property
from typing import List, Dict, Optional, Set, TypeVar, Generic, Iterable

from tracetools_analysis.processor.ros2 import Ros2Handler

from .utils import df_to_type_list

IdxItemType = TypeVar("IdxItemType")
Timestamp = namedtuple("Timestamp", ["timestamp"])


class Index(Generic[IdxItemType]):
    def __init__(self, items: Iterable[IdxItemType], **idx_fields):
        sort_key = lambda item: item.timestamp

        self.__items = list(items)
        self.__items.sort(key=sort_key)
        self.__indices = {}

        for idx_name, is_multi in idx_fields.items():
            index = {}
            self.__indices[idx_name] = index
            if is_multi:
                for item in self.__items:
                    key = getattr(item, idx_name)
                    if key not in index:
                        index[key] = []
                    index[key].append(item)  # Also sorted since items are processed in order and only filtered here
            else:  # Unique index
                duplicate_indices = defaultdict(lambda: 0)

                for item in self.__items:
                    key = getattr(item, idx_name)
                    if key in index:
                        duplicate_indices[key] += 1
                    index[key] = item

                if duplicate_indices:
                    print(f"[ENTKÄFERN] Duplicate Indices in {idx_name}:")
                # for key, count in duplicate_indices.items():
                #     print(f"--{key:<20d}'s last candidate: {repr(index[key])}")

    def __iter__(self):
        return iter(self.__items)

    def __len__(self):
        return len(self.__items)

    def __getattr__(self, item: str):
        if not item.startswith("by_"):
            return AttributeError(
                    f"Not found in index: '{item}'. Index lookups must be of the shape 'by_<index_field>'.")

        return self.__indices[item.removeprefix("by_")]

    def __getstate__(self):
        return vars(self)

    def __setstate__(self, state):
        vars(self).update(state)


@dataclass
class TrContext:
    nodes: Index['TrNode']
    publishers: Index['TrPublisher']
    subscriptions: Index['TrSubscription']
    timers: Index['TrTimer']
    timer_node_links: Index['TrTimerNodeLink']
    subscription_objects: Index['TrSubscriptionObject']
    callback_objects: Index['TrCallbackObject']
    callback_symbols: Index['TrCallbackSymbol']
    publish_instances: Index['TrPublishInstance']
    callback_instances: Index['TrCallbackInstance']
    topics: Index['TrTopic']
    _uuid: UUID

    def __init__(self, handler: Ros2Handler):
        print("[TrContext] Processing ROS 2 objects from traces...")

        self.nodes = Index(df_to_type_list(handler.data.nodes, TrNode, _c=self),
                           id=False)
        self.publishers = Index(df_to_type_list(handler.data.rcl_publishers, TrPublisher, _c=self),
                                id=False, node_handle=True, topic_name=True)
        self.subscriptions = Index(df_to_type_list(handler.data.rcl_subscriptions, TrSubscription, _c=self),
                                   id=False, node_handle=True, topic_name=True)
        self.timers = Index(df_to_type_list(handler.data.timers, TrTimer, _c=self),
                            id=False)
        self.timer_node_links = Index(df_to_type_list(handler.data.timer_node_links, TrTimerNodeLink),
                                      id=False, node_handle=True)
        self.subscription_objects = Index(
                df_to_type_list(handler.data.subscription_objects, TrSubscriptionObject, _c=self),
                id=False, subscription_handle=False)
        self.callback_objects = Index(df_to_type_list(handler.data.callback_objects, TrCallbackObject, _c=self),
                                      id=False, callback_object=False)
        self.callback_symbols = Index(df_to_type_list(handler.data.callback_symbols, TrCallbackSymbol, _c=self),
                                      id=False)
        self.publish_instances = Index(df_to_type_list(handler.data.rcl_publish_instances, TrPublishInstance, _c=self,
                                                       mappers={"timestamp": lambda t: t * 1e-9}),
                                       publisher_handle=True)
        self.callback_instances = Index(df_to_type_list(handler.data.callback_instances, TrCallbackInstance, _c=self,
                                                        mappers={"timestamp": lambda t: t.timestamp(),
                                                                 "duration": lambda d: d.total_seconds()}),
                                        callback_object=True)

        _unique_topic_names = {*(pub.topic_name for pub in self.publishers),
                               *(sub.topic_name for sub in self.subscriptions)}

        self.topics = Index((TrTopic(name=name, _c=self) for name in _unique_topic_names),
                            name=False)

        self._uuid = uuid4()

    def __hash__(self):
        return hash(self._uuid)

    def __eq__(self, other):
        return isinstance(other, TrContext) and other._uuid == self._uuid

    def __repr__(self):
        return f"TrContext"


@dataclass
class TrNode:
    id: int
    timestamp: int
    tid: int
    rmw_handle: int
    name: str
    namespace: str
    _c: TrContext = field(repr=False)

    @property
    def path(self) -> str:
        return '/'.join((self.namespace, self.name)).replace('//', '/')

    @property
    def publishers(self) -> List['TrPublisher']:
        return self._c.publishers.by_node_handle.get(self.id) or []

    @property
    def subscriptions(self) -> List['TrSubscription']:
        return self._c.subscriptions.by_node_handle.get(self.id) or []

    @property
    def timers(self) -> List['TrTimer']:
        links = self._c.timer_node_links.by_node_handle.get(self.id) or []
        timers = [self._c.timers.by_id.get(link.id) for link in links]
        return [t for t in timers if t is not None]

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrPublisher:
    id: int
    timestamp: int
    node_handle: int
    rmw_handle: int
    topic_name: str
    depth: int
    _c: TrContext = field(repr=False)

    @property
    def node(self) -> Optional['TrNode']:
        return self._c.nodes.by_id.get(self.node_handle)

    @property
    def subscriptions(self) -> List['TrSubscription']:
        return self._c.subscriptions.by_topic_name.get(self.topic_name) or []

    @property
    def instances(self) -> List['TrPublishInstance']:
        return self._c.publish_instances.by_publisher_handle.get(self.id) or []

    @property
    def topic(self) -> Optional['TrTopic']:
        return self._c.topics.by_name.get(self.topic_name)

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrSubscription:
    id: int
    timestamp: int
    node_handle: int
    rmw_handle: int
    topic_name: str
    depth: int
    _c: TrContext = field(repr=False)

    @property
    def node(self) -> Optional['TrNode']:
        return self._c.nodes.by_id.get(self.node_handle)

    @property
    def publishers(self) -> List['TrPublisher']:
        return self._c.publishers.by_topic_name.get(self.topic_name) or []

    @property
    def subscription_object(self) -> Optional['TrSubscriptionObject']:
        return self._c.subscription_objects.by_subscription_handle.get(self.id)

    @property
    def topic(self) -> Optional['TrTopic']:
        return self._c.topics.by_name.get(self.topic_name)

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrTimer:
    id: int
    timestamp: int
    period: int
    tid: int
    _c: TrContext = field(repr=False)

    @property
    def node(self) -> Optional['TrNode']:
        link = self._c.timer_node_links.by_id.get(self.id)
        if link is None:
            return None
        return self._c.nodes.by_id.get(link.node_handle)

    @property
    def callback_object(self) -> Optional['TrCallbackObject']:
        return self._c.callback_objects.by_id.get(self.id)

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrTimerNodeLink:
    id: int
    timestamp: int
    node_handle: int

    def __hash__(self):
        return hash((self.id, self.node_handle))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrSubscriptionObject:
    id: int
    timestamp: int
    subscription_handle: int
    _c: TrContext = field(repr=False)

    @property
    def subscription(self) -> Optional['TrSubscription']:
        return self._c.subscriptions.by_id.get(self.subscription_handle)

    @property
    def callback_object(self) -> Optional['TrCallbackObject']:
        return self._c.callback_objects.by_id.get(self.id)

    def __hash__(self):
        return hash((self.id, self.timestamp, self.subscription_handle))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrCallbackObject:
    id: int  # (reference) = subscription_object.id | timer.id | ....
    timestamp: int
    callback_object: int
    _c: TrContext = field(repr=False)

    @property
    def callback_instances(self) -> List['TrCallbackInstance']:
        return self._c.callback_instances.by_callback_object.get(self.callback_object) or []

    @property
    def callback_symbol(self) -> Optional['TrCallbackSymbol']:
        return self._c.callback_symbols.by_id.get(self.callback_object)

    @property
    def owner(self):
        if self.id in self._c.timers.by_id:
            return self._c.timers.by_id[self.id]
        if self.id in self._c.publishers.by_id:
            return self._c.publishers.by_id[self.id]
        if self.id in self._c.subscription_objects.by_id:
            return self._c.subscription_objects.by_id[self.id]
        return None

    def __hash__(self):
        return hash((self.id, self.timestamp, self.callback_object))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrPublishInstance:
    publisher_handle: int
    timestamp: float
    message: int
    _c: TrContext = field(repr=False)

    @property
    def publisher(self) -> Optional['TrPublisher']:
        return self._c.publishers.by_id.get(self.publisher_handle)

    def __hash__(self):
        return hash((self.publisher_handle, self.timestamp, self.message))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrCallbackInstance:
    callback_object: int
    timestamp: float
    duration: float
    intra_process: bool
    _c: TrContext = field(repr=False)

    @property
    def callback_obj(self) -> Optional['TrCallbackObject']:
        return self._c.callback_objects.by_callback_object.get(self.callback_object)

    @property
    def t_start(self):
        return self.timestamp

    @cached_property
    def t_end(self):
        return self.timestamp + self.duration

    def __hash__(self):
        return hash((self.callback_object, self.timestamp, self.duration))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrCallbackSymbol:
    id: int  # callback_object
    timestamp: int
    symbol: str
    _c: TrContext = field(repr=False)

    @property
    def callback_obj(self) -> Optional['TrCallbackObject']:
        return self._c.callback_objects.by_callback_object.get(self.id)

    def __hash__(self):
        return hash((self.id, self.timestamp, self.symbol))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


#######################################
# Self-defined (not from ROS2DataModel)
#######################################

@dataclass
class TrTopic:
    name: str
    _c: TrContext = field(repr=False)
    timestamp: int = 0

    @property
    def publishers(self) -> List['TrPublisher']:
        return self._c.publishers.by_topic_name.get(self.name) or []

    @property
    def subscriptions(self) -> List['TrSubscription']:
        return self._c.subscriptions.by_topic_name.get(self.name) or []

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()
