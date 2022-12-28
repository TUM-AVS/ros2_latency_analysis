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
    """
    This class implements a sized, iterable collection that allows building indices on its content.
    These indices are accessed by `index.by_<index_field>(<lookup_value>)`.
    1-1, 1-n and n-m indices are supported. The indices are hash-, not range-based.
    `IdxItemType` is expected to have a `timestamp` field by which this collection sorts its items.
    """
    def __init__(self, items: Iterable[IdxItemType], **idx_fields):
        """
        Construct an index.
        :param items: An iterable of items to add to the index. Items have to have a `timestamp` field
        :param idx_fields: Keyword args of the shape `<field_name> = <False|True|'n-to-m'>`. For each `<field_name>`, an index (1 to 1 for `False`, 1 to n for `True`, n to m for `'n-to-m'` is generated which can be accessed by `index.by_<field_name>(<lookup_value)` later.
        """
        self.__idx_fields = idx_fields.copy()
        self.__items = None
        self.__indices = None

        self.rebuild(items)

    def rebuild(self, items: Optional[Iterable[IdxItemType]] = None):
        """
        Clears all items and index structure contents, inserts the given `items` and recomputes all indices defined in the constructor.
        :param items: The items to overwrite the index's contents. If `None`, the current items in the index remain
        """
        def sort_key(item):
            return item.timestamp

        # Keep self.__items if `None` is given as an argument, otherwise overwrite them.
        if items is not None:
            self.__items = list(items)
        # Sort items and clear built indices
        self.__items.sort(key=sort_key)
        self.__indices = {}

        # Build each index one-by-one that was requested in the constructor
        for idx_name, is_multi in self.__idx_fields.items():
            index = {}
            self.__indices[idx_name] = index

            if is_multi in (True, 'n-to-m'):  # Multi-index case (True = 1-n, 'n-to-m' = n-m)
                for item in self.__items:
                    if is_multi == 'n-to-m':
                        keys = getattr(item, idx_name)  # Returns a list of multiple keys (n-m)
                    else:
                        keys = [getattr(item, idx_name)]  # Only one key (1-n)

                    # Insert the item into the index for each of its keys
                    # Multi-indices return a list for every key
                    for key in keys:
                        if key not in index:
                            index[key] = []
                        # Also sorted since items are processed in order and only filtered here
                        index[key].append(item)
            elif not is_multi:  # Unique index
                duplicate_indices = defaultdict(lambda: 0)

                # Insert item into the index at its key.
                # Duplicates are accepted with a warning, although they should not exist.
                # ROS 2 tracing ID duplication etc. causes them to occur though.
                # This has not been observed to cause problems within the Autoware stack,
                # only some info on ROS 2-internal stuff is lost.
                for item in self.__items:
                    key = getattr(item, idx_name)
                    if key in index:
                        duplicate_indices[key] += 1
                    # Unique indices return a single item for every key
                    index[key] = item

                if duplicate_indices:
                    print(f"[DEBUG] Duplicate Indices in {idx_name}")
            else:  # No valid index type given
                raise ValueError(f"is_multi has to equal one of the following: (False, True, 'n-to-m') but is {is_multi}")

    def append(self, items: Iterable[IdxItemType]):
        """
        Append `items` to an existing index and insert them into the index structures defined in the constructor.
        :param items: The items to add to the collection
        """
        self.rebuild(list(self.__items) + list(items))

    def clear(self):
        """
        Clear all items and index structure contents. Keep index definitions for the case that items are added after this call.
        """
        self.rebuild([])

    def __iter__(self):
        return iter(self.__items)

    def __len__(self):
        return len(self.__items)

    def __getattr__(self, item: str):
        """
        Returns the index pointed at by `item`.
        The index is a function accepting one key, and returning either one item or a list of items depending on whether it is a unique or multi-index.
        :param item: The index name, preceded by `'by_'`.
        :return: The requested index function
        """
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
    """
    Contains all data that is needed to represent a tracing session.
    The contained data is postprocessed, interlinked and indexed after being retrieved from a ros2_tracing `Ros2Handler`.
    """

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
        """
        Build the context from a `Ros2Handler` instance.
        :param handler: The `Ros2Handler` instance to build the context from
        """
        print("[TrContext] Processing ROS 2 objects from traces...")

        self.nodes = Index(df_to_type_list(handler.data.nodes, TrNode, _c=self),
                           id=False)
        self.publishers = Index(df_to_type_list(handler.data.rcl_publishers, TrPublisher, _c=self),
                                id=False, node_handle=True, topic_name=True)
        self.subscriptions = Index(df_to_type_list(handler.data.rcl_subscriptions, TrSubscription,
                                                   column_value_mappers={"topic_name": lambda n: [n]},
                                                   column_to_field_mappings={"topic_name": "topic_names"},
                                                   _c=self),
                                   id=False, node_handle=True, topic_names='n-to-m')
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
                                                       column_value_mappers={"timestamp": lambda t: t * 1e-9}),
                                       publisher_handle=True)
        self.callback_instances = Index(df_to_type_list(handler.data.callback_instances, TrCallbackInstance, _c=self,
                                                        column_value_mappers={"timestamp": lambda t: t.timestamp(),
                                                                 "duration": lambda d: d.total_seconds()}),
                                        callback_object=True)

        _unique_topic_names = {*(pub.topic_name for pub in self.publishers),
                               *(n for sub in self.subscriptions for n in sub.topic_names)}

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
    """
    The representation of a ROS 2 node in the tracing context.
    """
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
    """
    The representation of a ROS 2 publisher in the tracing context.
    """
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
        return self._c.subscriptions.by_topic_names.get(self.topic_name) or []

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
    """
    The representation of a ROS 2 subscription in the tracing context.
    """
    id: int
    timestamp: int
    node_handle: int
    rmw_handle: int
    topic_names: List[str]
    depth: int
    _c: TrContext = field(repr=False)

    @property
    def node(self) -> Optional['TrNode']:
        return self._c.nodes.by_id.get(self.node_handle)

    @property
    def publishers(self) -> List['TrPublisher']:
        return list(set(p for n in self.topic_names for p in self._c.publishers.by_topic_name.get(n)))

    @property
    def subscription_object(self) -> Optional['TrSubscriptionObject']:
        return self._c.subscription_objects.by_subscription_handle.get(self.id)

    @property
    def topics(self) -> List['TrTopic']:
        return list(set(self._c.topics.by_name.get(n) for n in self.topic_names))

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrTimer:
    """
    The representation of a ROS 2 timer in the tracing context.
    """
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
    """
    The relation connecting timers to nodes in ROS 2 tracing data.
    """
    id: int
    timestamp: int
    node_handle: int

    def __hash__(self):
        return hash((self.id, self.node_handle))

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()


@dataclass
class TrSubscriptionObject:
    """
    The relation connecting subscriptions to callback objects to nodes in ROS 2 tracing data.
    """
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
    """
    The relation connecting callback instances to subscriptions/timers/etc. in ROS 2 tracing data.
    """
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
    """
    A publication of a message in ROS 2 tracing data.
    """
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
    """
    An invocation of a callback (from a subscription or timer) in ROS 2 tracing data.
    """
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
    """
    The C++ symbol corresponding to a callback in ROS 2 tracing data.
    This is typically a very convoluted name with lots of C++ wrappers and almost no symbols or identifiers preserved.
    Use `repr(matching.subscriptions.sanitize(tr_callback_symbol.symbol))` to get a readable name.
    """
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
    """
    The representation of a ROS 2 topic, linking publishers and subscriptions.
    """
    name: str
    _c: TrContext = field(repr=False)
    timestamp: int = 0

    @property
    def publishers(self) -> List['TrPublisher']:
        return self._c.publishers.by_topic_name.get(self.name) or []

    @property
    def subscriptions(self) -> List['TrSubscription']:
        return self._c.subscriptions.by_topic_names.get(self.name) or []

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()
