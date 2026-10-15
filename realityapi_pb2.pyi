from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class RequestType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    HEARTBEAT: _ClassVar[RequestType]
    REQUEST_ALL_DATA: _ClassVar[RequestType]
    TURTLE_INFO: _ClassVar[RequestType]
    DOG_INFO: _ClassVar[RequestType]
HEARTBEAT: RequestType
REQUEST_ALL_DATA: RequestType
TURTLE_INFO: RequestType
DOG_INFO: RequestType

class Vector3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class TurtleInfo(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class TurtleCommand(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class DogInfo(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class DogCommand(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class DeviceInfo(_message.Message):
    __slots__ = ("id", "turtle", "dog")
    ID_FIELD_NUMBER: _ClassVar[int]
    TURTLE_FIELD_NUMBER: _ClassVar[int]
    DOG_FIELD_NUMBER: _ClassVar[int]
    id: str
    turtle: TurtleInfo
    dog: DogInfo
    def __init__(self, id: _Optional[str] = ..., turtle: _Optional[_Union[TurtleInfo, _Mapping]] = ..., dog: _Optional[_Union[DogInfo, _Mapping]] = ...) -> None: ...

class AllDataResponse(_message.Message):
    __slots__ = ("devices",)
    DEVICES_FIELD_NUMBER: _ClassVar[int]
    devices: _containers.RepeatedCompositeFieldContainer[DeviceInfo]
    def __init__(self, devices: _Optional[_Iterable[_Union[DeviceInfo, _Mapping]]] = ...) -> None: ...

class ServerBoundPacket(_message.Message):
    __slots__ = ("id", "timestamp", "req_type", "turtle", "dog")
    ID_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    REQ_TYPE_FIELD_NUMBER: _ClassVar[int]
    TURTLE_FIELD_NUMBER: _ClassVar[int]
    DOG_FIELD_NUMBER: _ClassVar[int]
    id: str
    timestamp: float
    req_type: RequestType
    turtle: TurtleInfo
    dog: DogInfo
    def __init__(self, id: _Optional[str] = ..., timestamp: _Optional[float] = ..., req_type: _Optional[_Union[RequestType, str]] = ..., turtle: _Optional[_Union[TurtleInfo, _Mapping]] = ..., dog: _Optional[_Union[DogInfo, _Mapping]] = ...) -> None: ...

class ClientBoundPacket(_message.Message):
    __slots__ = ("id", "timestamp", "status", "all_data_res", "turtle_cmd", "dog_cmd")
    ID_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    ALL_DATA_RES_FIELD_NUMBER: _ClassVar[int]
    TURTLE_CMD_FIELD_NUMBER: _ClassVar[int]
    DOG_CMD_FIELD_NUMBER: _ClassVar[int]
    id: str
    timestamp: float
    status: int
    all_data_res: AllDataResponse
    turtle_cmd: TurtleCommand
    dog_cmd: DogCommand
    def __init__(self, id: _Optional[str] = ..., timestamp: _Optional[float] = ..., status: _Optional[int] = ..., all_data_res: _Optional[_Union[AllDataResponse, _Mapping]] = ..., turtle_cmd: _Optional[_Union[TurtleCommand, _Mapping]] = ..., dog_cmd: _Optional[_Union[DogCommand, _Mapping]] = ...) -> None: ...
