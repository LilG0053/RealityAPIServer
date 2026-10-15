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

class RawLidarScan(_message.Message):
    __slots__ = ("id", "transform", "ranges", "range_min", "range_max", "angle_min", "angle_increment")
    ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    RANGES_FIELD_NUMBER: _ClassVar[int]
    RANGE_MIN_FIELD_NUMBER: _ClassVar[int]
    RANGE_MAX_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MIN_FIELD_NUMBER: _ClassVar[int]
    ANGLE_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    id: str
    transform: Transform
    ranges: _containers.RepeatedScalarFieldContainer[float]
    range_min: float
    range_max: float
    angle_min: float
    angle_increment: float
    def __init__(self, id: _Optional[str] = ..., transform: _Optional[_Union[Transform, _Mapping]] = ..., ranges: _Optional[_Iterable[float]] = ..., range_min: _Optional[float] = ..., range_max: _Optional[float] = ..., angle_min: _Optional[float] = ..., angle_increment: _Optional[float] = ...) -> None: ...
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
    __slots__ = ("req_uuid", "post_device_info", "post_heartbeat", "fetch_all_device_info", "fetch_single_device_info", "send_command", "fetch_lidar_data", "post_raw_lidar_scan")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    POST_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_HEARTBEAT_FIELD_NUMBER: _ClassVar[int]
    FETCH_ALL_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    FETCH_SINGLE_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    SEND_COMMAND_FIELD_NUMBER: _ClassVar[int]
    FETCH_LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    POST_RAW_LIDAR_SCAN_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    post_device_info: DeviceInfo
    post_heartbeat: str
    fetch_all_device_info: _empty_pb2.Empty
    fetch_single_device_info: str
    send_command: Command
    fetch_lidar_data: _empty_pb2.Empty
    post_raw_lidar_scan: RawLidarScan
    def __init__(self, req_uuid: _Optional[str] = ..., post_device_info: _Optional[_Union[DeviceInfo, _Mapping]] = ..., post_heartbeat: _Optional[str] = ..., fetch_all_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., fetch_single_device_info: _Optional[str] = ..., send_command: _Optional[_Union[Command, _Mapping]] = ..., fetch_lidar_data: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., post_raw_lidar_scan: _Optional[_Union[RawLidarScan, _Mapping]] = ...) -> None: ...

class ClientRequest(_message.Message):
    __slots__ = ("req_uuid", "fetch_device_info", "post_command")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    FETCH_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_COMMAND_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    fetch_device_info: _empty_pb2.Empty
    post_command: Command
    def __init__(self, req_uuid: _Optional[str] = ..., fetch_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., post_command: _Optional[_Union[Command, _Mapping]] = ...) -> None: ...
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
