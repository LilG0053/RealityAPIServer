from google.protobuf import empty_pb2 as _empty_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class DeviceType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    NONE: _ClassVar[DeviceType]
    VR: _ClassVar[DeviceType]
    AR: _ClassVar[DeviceType]
    DOG: _ClassVar[DeviceType]
    ARM: _ClassVar[DeviceType]
    TURTLE: _ClassVar[DeviceType]
NONE: DeviceType
VR: DeviceType
AR: DeviceType
DOG: DeviceType
ARM: DeviceType
TURTLE: DeviceType

class Vector3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class Transform(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class Lidar(_message.Message):
    __slots__ = ("points",)
    POINTS_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[Vector3]
    def __init__(self, points: _Optional[_Iterable[_Union[Vector3, _Mapping]]] = ...) -> None: ...

class DeviceInfo(_message.Message):
    __slots__ = ("id", "device_type", "timestamp", "transform", "lidar")
    ID_FIELD_NUMBER: _ClassVar[int]
    DEVICE_TYPE_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    LIDAR_FIELD_NUMBER: _ClassVar[int]
    id: str
    device_type: DeviceType
    timestamp: float
    transform: Transform
    lidar: Lidar
    def __init__(self, id: _Optional[str] = ..., device_type: _Optional[_Union[DeviceType, str]] = ..., timestamp: _Optional[float] = ..., transform: _Optional[_Union[Transform, _Mapping]] = ..., lidar: _Optional[_Union[Lidar, _Mapping]] = ...) -> None: ...

class AllDeviceInfo(_message.Message):
    __slots__ = ("devices",)
    DEVICES_FIELD_NUMBER: _ClassVar[int]
    devices: _containers.RepeatedCompositeFieldContainer[DeviceInfo]
    def __init__(self, devices: _Optional[_Iterable[_Union[DeviceInfo, _Mapping]]] = ...) -> None: ...

class Response(_message.Message):
    __slots__ = ("req_uuid", "ok", "err_id_not_found", "err_not_supported", "err_cannot_execute", "err_failed_execute", "all_device_info", "device_info", "lidar_data")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    ERR_ID_NOT_FOUND_FIELD_NUMBER: _ClassVar[int]
    ERR_NOT_SUPPORTED_FIELD_NUMBER: _ClassVar[int]
    ERR_CANNOT_EXECUTE_FIELD_NUMBER: _ClassVar[int]
    ERR_FAILED_EXECUTE_FIELD_NUMBER: _ClassVar[int]
    ALL_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    ok: _empty_pb2.Empty
    err_id_not_found: _empty_pb2.Empty
    err_not_supported: _empty_pb2.Empty
    err_cannot_execute: _empty_pb2.Empty
    err_failed_execute: _empty_pb2.Empty
    all_device_info: AllDeviceInfo
    device_info: DeviceInfo
    lidar_data: Lidar
    def __init__(self, req_uuid: _Optional[str] = ..., ok: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_id_not_found: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_not_supported: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_cannot_execute: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_failed_execute: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., all_device_info: _Optional[_Union[AllDeviceInfo, _Mapping]] = ..., device_info: _Optional[_Union[DeviceInfo, _Mapping]] = ..., lidar_data: _Optional[_Union[Lidar, _Mapping]] = ...) -> None: ...

class Command(_message.Message):
    __slots__ = ("target_id", "transform")
    TARGET_ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    target_id: str
    transform: Transform
    def __init__(self, target_id: _Optional[str] = ..., transform: _Optional[_Union[Transform, _Mapping]] = ...) -> None: ...

class ServerRequest(_message.Message):
    __slots__ = ("req_uuid", "post_device_info", "post_heartbeat", "fetch_all_device_info", "fetch_single_device_info", "send_command", "fetch_lidar_data")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    POST_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_HEARTBEAT_FIELD_NUMBER: _ClassVar[int]
    FETCH_ALL_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    FETCH_SINGLE_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    SEND_COMMAND_FIELD_NUMBER: _ClassVar[int]
    FETCH_LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    post_device_info: DeviceInfo
    post_heartbeat: str
    fetch_all_device_info: _empty_pb2.Empty
    fetch_single_device_info: str
    send_command: Command
    fetch_lidar_data: _empty_pb2.Empty
    def __init__(self, req_uuid: _Optional[str] = ..., post_device_info: _Optional[_Union[DeviceInfo, _Mapping]] = ..., post_heartbeat: _Optional[str] = ..., fetch_all_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., fetch_single_device_info: _Optional[str] = ..., send_command: _Optional[_Union[Command, _Mapping]] = ..., fetch_lidar_data: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ...) -> None: ...

class ClientRequest(_message.Message):
    __slots__ = ("req_uuid", "fetch_device_info", "post_command")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    FETCH_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_COMMAND_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    fetch_device_info: _empty_pb2.Empty
    post_command: Command
    def __init__(self, req_uuid: _Optional[str] = ..., fetch_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., post_command: _Optional[_Union[Command, _Mapping]] = ...) -> None: ...
