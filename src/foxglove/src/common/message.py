import abc
from dataclasses import dataclass
import google.protobuf.message

@dataclass
class OutMessage:
    channel: str
    msg: bytes | google.protobuf.message.Message
    timestamp_ns: int

@dataclass
class InMessage:
    topic: str
    msg: bytes
    timestamp_ns: int