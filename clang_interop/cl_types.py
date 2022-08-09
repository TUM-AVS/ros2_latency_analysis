import os
import re
from dataclasses import dataclass, field
from typing import List, Literal, Dict, Set


@dataclass
class ClTranslationUnit:
    filename: str

    def __hash__(self):
        return hash(self.filename)


@dataclass
class ClContext:
    translation_units: Set['ClTranslationUnit']

    nodes: Set['ClNode']
    publishers: Set['ClPublisher']
    subscriptions: Set['ClSubscription']
    timers: Set['ClTimer']

    fields: Set['ClField']
    methods: Set['ClMethod']

    accesses: List['ClMemberRef']

    dependencies: Dict['ClMethod', Set['ClMethod']]
    publications: Dict['ClMethod', Set['ClPublisher']]

    def __repr__(self):
        return f"ClContext({len(self.translation_units)} TUs)"


@dataclass
class ClSourceRange:
    start_file: str
    start_line: int | None
    start_col: int | None

    end_file: str
    end_line: int | None
    end_col: int | None

    def __init__(self, json_obj):
        begin = json_obj["begin"].split(":")
        end = json_obj["end"].split(":")

        self.start_file = os.path.realpath(begin[0])
        self.start_line = int(begin[1]) if len(begin) > 1 else None
        self.start_col = int(begin[2].split(" ")[0]) if len(begin) > 2 else None

        self.end_file = os.path.realpath(end[0])
        self.end_line = int(end[1]) if len(end) > 1 else None
        self.end_col = int(end[2].split(" ")[0]) if len(end) > 2 else None

    def __hash__(self):
        return hash((self.start_file, self.start_line, self.start_col,
                     self.end_file, self.end_line, self.end_col))


@dataclass
class ClNode:
    tu: 'ClTranslationUnit' = field(repr=False)
    id: int
    qualified_name: str
    source_range: 'ClSourceRange' = field(repr=False)
    field_ids: List[int] | None
    method_ids: List[int] | None
    ros_name: str | None
    ros_namespace: str | None

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.id = json_obj['id']
        self.qualified_name = json_obj['qualified_name']
        self.source_range = ClSourceRange(json_obj['source_range'])
        self.field_ids = list(map(lambda obj: obj['id'], json_obj['fields'])) if 'fields' in json_obj else None
        self.method_ids = list(map(lambda obj: obj['id'], json_obj['methods'])) if 'methods' in json_obj else None
        self.ros_name = json_obj['ros_name'] if 'ros_name' in json_obj else None
        self.ros_namespace = json_obj['ros_namespace'] if 'ros_namespace' in json_obj else None

    def __hash__(self):
        return hash((self.tu, self.id))


@dataclass
class ClMethod:
    tu: 'ClTranslationUnit' = field(repr=False)
    id: int
    qualified_name: str
    source_range: 'ClSourceRange' = field(repr=False)
    return_type: str | None
    parameter_types: List[str] | None
    is_lambda: bool | None

    @property
    def signature(self):
        # Lambda definitions end in this suffix
        class_name = self.qualified_name.removesuffix("::(anonymous class)::operator()")

        # If the definition is no lambda (and hence no suffix has been removed), the last part after :: is the method
        # name. Remove it to get the class name.
        if class_name == self.qualified_name:
            class_name = "::".join(class_name.split("::")[:-1])

        if self.is_lambda:
            return f"{class_name}$lambda"

        param_str = ','.join(self.parameter_types) if self.parameter_types is not None else ''
        return f"{self.return_type if self.return_type else ''} ({class_name})({param_str})"

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.id = json_obj['id']
        self.qualified_name = json_obj['qualified_name']
        self.source_range = ClSourceRange(json_obj['source_range'])
        self.return_type = json_obj['signature']['return_type'] if 'signature' in json_obj else None
        self.parameter_types = json_obj['signature']['parameter_types'] if 'signature' in json_obj else None
        self.is_lambda = json_obj['is_lambda'] if 'is_lambda' in json_obj else None

    def __hash__(self):
        return hash(self.id)


@dataclass
class ClField:
    tu: 'ClTranslationUnit' = field(repr=False)
    id: int
    qualified_name: str
    source_range: 'ClSourceRange' = field(repr=False)

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.id = json_obj['id']
        self.qualified_name = json_obj['qualified_name']
        self.source_range = ClSourceRange(json_obj['source_range'])

    def __hash__(self):
        return hash(self.id)


@dataclass
class ClMemberRef:
    tu: 'ClTranslationUnit' = field(repr=False)
    type: Literal["read", "write", "call", "arg", "pub"] | None
    member_chain: List[int]
    method_id: int | None
    node_id: int | None
    source_range: 'ClSourceRange' = field(repr=False)

    def __init__(self, json_obj, tu):
        self.tu = tu
        access_type = json_obj['context']['access_type']
        if access_type == 'none':
            access_type = None
        self.type = access_type
        self.member_chain = list(map(lambda obj: obj['id'], json_obj['member'][::-1]))
        self.method_id = json_obj['context']['method']['id'] if 'method' in json_obj['context'] else None
        self.node_id = json_obj['context']['node']['id'] if 'node' in json_obj['context'] else None
        self.source_range = ClSourceRange(json_obj['context']['statement']['source_range'])

    def __hash__(self):
        return self.source_range.__hash__()


@dataclass
class ClSubscription:
    tu: 'ClTranslationUnit' = field(repr=False)
    topic: str | None
    callback_id: int | None
    source_range: 'ClSourceRange' = field(repr=False)

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.topic = json_obj['topic'] if 'topic' in json_obj else None
        self.callback_id = json_obj['callback']['id'] if 'callback' in json_obj else None
        self.source_range = ClSourceRange(json_obj['source_range'])

    def __hash__(self):
        return self.source_range.__hash__()


@dataclass
class ClPublisher:
    tu: 'ClTranslationUnit' = field(repr=False)
    topic: str | None
    member_id: int | None
    source_range: 'ClSourceRange' = field(repr=False)

    def update(self, t2: 'ClTimer'):
        return self

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.topic = json_obj['topic'] if 'topic' in json_obj else None
        self.member_id = json_obj['member']['id'] if 'member' in json_obj else None
        self.source_range = ClSourceRange(json_obj['source_range'])

    def __hash__(self):
        return self.source_range.__hash__()


@dataclass
class ClTimer:
    tu: 'ClTranslationUnit' = field(repr=False)
    callback_id: int | None
    source_range: 'ClSourceRange' = field(repr=False)

    def __init__(self, json_obj, tu):
        self.tu = tu
        self.callback_id = json_obj['callback']['id'] if 'callback' in json_obj else None
        self.source_range = ClSourceRange(json_obj['source_range'])

    def __hash__(self):
        return self.source_range.__hash__()
