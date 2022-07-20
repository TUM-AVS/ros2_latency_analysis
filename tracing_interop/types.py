from dataclasses import dataclass
from functools import cached_property
from typing import List, Dict

import pandas as pd

from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

from .utils import list_to_dict, df_to_type_list


@dataclass
class TrContext:
    nodes: Dict[int, 'TrNode']
    publishers: Dict[int, 'TrPublisher']
    subscriptions: Dict[int, 'TrSubscription']
    timers: Dict[int, 'TrTimer']
    timer_node_links: Dict[int, 'TrTimerNodeLink']
    subscription_objects: Dict[int, 'TrSubscriptionObject']
    callback_objects: Dict[int, 'TrCallbackObject']
    callback_symbols: Dict[int, 'TrCallbackSymbol']
    publish_instances: List['TrPublishInstance']
    callback_instances: List['TrCallbackInstance']
    topics: Dict[str, 'TrTopic']

    util: Ros2DataModelUtil | None
    handler: Ros2Handler | None

    def __init__(self, util: Ros2DataModelUtil, handler: Ros2Handler):
        self.util = util
        self.handler = handler

        print("[TrContext] Processing ROS 2 objects from traces...")

        self.nodes = list_to_dict(df_to_type_list(handler.data.nodes, TrNode, _c=self))
        print(f" ├─ Processed {len(self.nodes):<8d} nodes")
        self.publishers = list_to_dict(df_to_type_list(handler.data.rcl_publishers, TrPublisher, _c=self))
        print(f" ├─ Processed {len(self.publishers):<8d} publishers")
        self.subscriptions = list_to_dict(df_to_type_list(handler.data.rcl_subscriptions, TrSubscription, _c=self))
        print(f" ├─ Processed {len(self.subscriptions):<8d} subscriptions")
        self.timers = list_to_dict(df_to_type_list(handler.data.timers, TrTimer, _c=self))
        print(f" ├─ Processed {len(self.timers):<8d} timers")
        self.timer_node_links = list_to_dict(df_to_type_list(handler.data.timer_node_links, TrTimerNodeLink))
        print(f" ├─ Processed {len(self.timer_node_links):<8d} timer-node links")
        self.subscription_objects = list_to_dict(
            df_to_type_list(handler.data.subscription_objects, TrSubscriptionObject, _c=self))
        print(f" ├─ Processed {len(self.subscription_objects):<8d} subscription objects")
        self.callback_objects = list_to_dict(df_to_type_list(handler.data.callback_objects, TrCallbackObject, _c=self))
        print(f" ├─ Processed {len(self.callback_objects):<8d} callback objects")
        self.callback_symbols = list_to_dict(df_to_type_list(handler.data.callback_symbols, TrCallbackSymbol, _c=self))
        print(f" ├─ Processed {len(self.callback_symbols):<8d} callback symbols")
        self.publish_instances = df_to_type_list(handler.data.rcl_publish_instances, TrPublishInstance, _c=self)
        print(f" ├─ Processed {len(self.publish_instances):<8d} publish instances")
        self.callback_instances = df_to_type_list(handler.data.callback_instances, TrCallbackInstance, _c=self)
        print(f" ├─ Processed {len(self.callback_instances):<8d} callback instances")

        _unique_topic_names = {*(pub.topic_name for pub in self.publishers.values()),
                               *(sub.topic_name for sub in self.subscriptions.values())}
        self.topics = list_to_dict(map(lambda name: TrTopic(name=name, _c=self), _unique_topic_names), key="name")
        print(f" └─ Processed {len(self.topics):<8d} topics\n")

        print("[TrContext] Caching dynamic properties...")

        [(o.path, o.publishers, o.subscriptions, o.timers) for o in self.nodes.values()]
        print(" ├─ Cached node properties")
        [(o.instances, o.subscriptions) for o in self.publishers.values()]
        print(" ├─ Cached publisher properties")
        [(o.publishers, o.subscription_objects) for o in self.subscriptions.values()]
        print(" ├─ Cached subscription properties")
        [(o.nodes) for o in self.timers.values()]
        print(" ├─ Cached timer properties")
        [(o.callback_instances, o.owner, o.owner_info) for o in self.callback_objects.values()]
        print(" ├─ Cached callback object properties")
        [(o.callback_objs) for o in self.callback_symbols.values()]
        print(" ├─ Cached callback symbol properties")
        [(o.publishers, o.subscriptions) for o in self.topics.values()]
        print(" └─ Cached topic properties\n")

    def __getstate__(self):
        state = self.__dict__.copy()
        del state["util"]
        del state["handler"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self.util = None
        self.handler = None


@dataclass
class TrNode:
    id: int
    timestamp: int
    tid: int
    rmw_handle: int
    name: str
    namespace: str
    _c: TrContext

    @cached_property
    def path(self) -> str:
        return '/'.join((self.namespace, self.name))

    @cached_property
    def publishers(self) -> List['TrPublisher']:
        return list(filter(lambda pub: pub.node_handle == self.id, self._c.publishers.values()))

    @cached_property
    def subscriptions(self) -> List['TrSubscription']:
        return list(filter(lambda sub: sub.node_handle == self.id, self._c.subscriptions.values()))

    @cached_property
    def timers(self) -> List['TrTimer']:
        links = [link.id for link in self._c.timer_node_links.values() if link.node_handle == self.id]
        return list(filter(lambda timer: timer.id in links, self._c.timers.values()))

    def __hash__(self):
        return hash(self.id)


@dataclass
class TrPublisher:
    id: int
    timestamp: int
    node_handle: int
    rmw_handle: int
    topic_name: str
    depth: int
    _c: TrContext

    @property
    def node(self) -> 'TrNode':
        return self._c.nodes[self.node_handle]

    @cached_property
    def subscriptions(self) -> List['TrSubscription']:
        return list(filter(lambda sub: sub.topic_name == self.topic_name, self._c.subscriptions.values()))

    @cached_property
    def instances(self) -> List['TrPublishInstance']:
        return list(filter(lambda inst: inst.publisher_handle == self.id, self._c.publish_instances))

    @property
    def topic(self) -> 'TrTopic':
        return self._c.topics[self.topic_name]

    def __hash__(self):
        return hash(self.id)


@dataclass
class TrSubscription:
    id: int
    timestamp: int
    node_handle: int
    rmw_handle: int
    topic_name: str
    depth: int
    _c: TrContext

    @property
    def node(self) -> 'TrNode':
        return self._c.nodes[self.node_handle]

    @cached_property
    def publishers(self) -> List['TrPublisher']:
        return list(filter(lambda pub: pub.topic_name == self.topic_name, self._c.publishers.values()))

    @cached_property
    def subscription_objects(self) -> List['TrSubscriptionObject']:
        return list(
                filter(lambda sub_obj: sub_obj.subscription_handle == self.id, self._c.subscription_objects.values()))

    @property
    def topic(self) -> 'TrTopic':
        return self._c.topics[self.topic_name]

    def __hash__(self):
        return hash(self.id)


@dataclass
class TrTimer:
    id: int
    timestamp: int
    period: int
    tid: int
    _c: TrContext

    @cached_property
    def nodes(self) -> List['TrNode']:
        links = [link.node_handle for link in self._c.timer_node_links.values() if link.id == self.id]
        return list(filter(lambda node: node.id in links, self._c.nodes.values()))

    @property
    def callback_object(self) -> 'TrCallbackObject':
        return self._c.callback_objects[self.id]

    def __hash__(self):
        return hash(self.id)


@dataclass
class TrTimerNodeLink:
    id: int
    timestamp: int
    node_handle: int


@dataclass
class TrSubscriptionObject:
    id: int  # subscription
    timestamp: int
    subscription_handle: int
    _c: TrContext

    @property
    def subscription(self) -> 'TrSubscription':
        return self._c.subscriptions[self.subscription_handle]

    @property
    def callback_object(self) -> 'TrCallbackObject':
        return self._c.callback_objects[self.id]

    def __hash__(self):
        return hash((self.id, self.timestamp, self.subscription_handle))


@dataclass
class TrCallbackObject:
    id: int  # (reference) = subscription_object.id | timer.id | ....
    timestamp: int
    callback_object: int
    _c: TrContext

    @cached_property
    def callback_instances(self) -> List['TrCallbackInstance']:
        return list(filter(lambda inst: inst.callback_object == self.callback_object, self._c.callback_instances))

    @property
    def callback_symbol(self) -> 'TrCallbackSymbol':
        return self._c.callback_symbols[self.id]

    @cached_property
    def owner(self):
        if self.id in self._c.timers:
            return self._c.timers[self.id]
        if self.id in self._c.publishers:
            return self._c.publishers[self.id]
        if self.id in self._c.subscription_objects:
            return self._c.subscription_objects[self.id]
        if self.id in self._c.handler.data.services.index:
            return 'Service'
        if self.id in self._c.handler.data.clients.index:
            return 'Client'
        return None

    @cached_property
    def owner_info(self):
        info = self._c.util.get_callback_owner_info(self.callback_object)
        if info is None:
            return None, None

        type_name, dict_str = info.split(" -- ")
        kv_strs = dict_str.split(", ")
        info_dict = {k: v for k, v in map(lambda kv_str: kv_str.split(": ", maxsplit=1), kv_strs)}
        return type_name, info_dict

    def __hash__(self):
        return hash((self.id, self.timestamp, self.callback_object))


@dataclass
class TrPublishInstance:
    publisher_handle: int
    timestamp: int
    message: int
    _c: TrContext

    @property
    def publisher(self) -> 'TrPublisher':
        return self._c.publishers[self.publisher_handle]

    def __hash__(self):
        return hash((self.publisher_handle, self.timestamp, self.message))


@dataclass
class TrCallbackInstance:
    callback_object: int
    timestamp: pd.Timestamp
    duration: pd.Timedelta
    intra_process: bool
    _c: TrContext

    @property
    def callback_obj(self) -> 'TrCallbackObject':
        return self._c.callback_objects[self.callback_object]

    def __hash__(self):
        return hash((self.callback_object, self.timestamp, self.duration))


@dataclass
class TrCallbackSymbol:
    id: int  # callback_object
    timestamp: int
    symbol: str
    _c: TrContext

    @cached_property
    def callback_objs(self) -> List['TrCallbackObject']:
        return list(filter(lambda cb_obj: cb_obj.callback_object == self.id, self._c.callback_objects.values()))

    def __hash__(self):
        return hash((self.id, self.timestamp, self.symbol))


#######################################
# Self-defined (not from ROS2DataModel)
#######################################

@dataclass
class TrTopic:
    name: str
    _c: TrContext

    @cached_property
    def publishers(self) -> List['TrPublisher']:
        return list(filter(lambda pub: pub.topic_name == self.name, self._c.publishers.values()))

    @cached_property
    def subscriptions(self) -> List['TrSubscription']:
        return list(filter(lambda sub: sub.topic_name == self.name, self._c.subscriptions.values()))

    def __hash__(self):
        return hash(self.name)
