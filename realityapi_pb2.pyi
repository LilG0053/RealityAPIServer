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

class Vector2(_message.Message):
    __slots__ = ("x", "y")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ...) -> None: ...

class Transform2(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector2
    rotation: float
    def __init__(self, position: _Optional[_Union[Vector2, _Mapping]] = ..., rotation: _Optional[float] = ...) -> None: ...

class Polygon2(_message.Message):
    __slots__ = ("vertices",)
    VERTICES_FIELD_NUMBER: _ClassVar[int]
    vertices: _containers.RepeatedCompositeFieldContainer[Vector2]
    def __init__(self, vertices: _Optional[_Iterable[_Union[Vector2, _Mapping]]] = ...) -> None: ...

class Vector3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class Transform3(_message.Message):
    __slots__ = ("position", "rotation")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    position: Vector3
    rotation: Vector3
    def __init__(self, position: _Optional[_Union[Vector3, _Mapping]] = ..., rotation: _Optional[_Union[Vector3, _Mapping]] = ...) -> None: ...

class Lidar2(_message.Message):
    __slots__ = ("points",)
    POINTS_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[Vector2]
    def __init__(self, points: _Optional[_Iterable[_Union[Vector2, _Mapping]]] = ...) -> None: ...

class Lidar3(_message.Message):
    __slots__ = ("points",)
    POINTS_FIELD_NUMBER: _ClassVar[int]
    points: _containers.RepeatedCompositeFieldContainer[Vector3]
    def __init__(self, points: _Optional[_Iterable[_Union[Vector3, _Mapping]]] = ...) -> None: ...

class RawLidar2Scan(_message.Message):
    __slots__ = ("ranges", "range_min", "range_max", "angle_min", "angle_increment")
    RANGES_FIELD_NUMBER: _ClassVar[int]
    RANGE_MIN_FIELD_NUMBER: _ClassVar[int]
    RANGE_MAX_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MIN_FIELD_NUMBER: _ClassVar[int]
    ANGLE_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    ranges: _containers.RepeatedScalarFieldContainer[float]
    range_min: float
    range_max: float
    angle_min: float
    angle_increment: float
    def __init__(self, ranges: _Optional[_Iterable[float]] = ..., range_min: _Optional[float] = ..., range_max: _Optional[float] = ..., angle_min: _Optional[float] = ..., angle_increment: _Optional[float] = ...) -> None: ...

class RawPointCloud(_message.Message):
    __slots__ = ("id", "transform", "xyz")
    ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    XYZ_FIELD_NUMBER: _ClassVar[int]
    id: str
    transform: Transform3
    xyz: bytes
    def __init__(self, id: _Optional[str] = ..., transform: _Optional[_Union[Transform3, _Mapping]] = ..., xyz: _Optional[bytes] = ...) -> None: ...

class RoomBounds(_message.Message):
    __slots__ = ("origin", "room", "obstacles")
    ORIGIN_FIELD_NUMBER: _ClassVar[int]
    ROOM_FIELD_NUMBER: _ClassVar[int]
    OBSTACLES_FIELD_NUMBER: _ClassVar[int]
    origin: Vector2
    room: Polygon2
    obstacles: _containers.RepeatedCompositeFieldContainer[Polygon2]
    def __init__(self, origin: _Optional[_Union[Vector2, _Mapping]] = ..., room: _Optional[_Union[Polygon2, _Mapping]] = ..., obstacles: _Optional[_Iterable[_Union[Polygon2, _Mapping]]] = ...) -> None: ...

class OccupancyCell(_message.Message):
    __slots__ = ("x", "z", "p")
    X_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    P_FIELD_NUMBER: _ClassVar[int]
    x: int
    z: int
    p: float
    def __init__(self, x: _Optional[int] = ..., z: _Optional[int] = ..., p: _Optional[float] = ...) -> None: ...

class OccupancyGrid(_message.Message):
    __slots__ = ("cell_size", "revision", "reset", "cells")
    CELL_SIZE_FIELD_NUMBER: _ClassVar[int]
    REVISION_FIELD_NUMBER: _ClassVar[int]
    RESET_FIELD_NUMBER: _ClassVar[int]
    CELLS_FIELD_NUMBER: _ClassVar[int]
    cell_size: float
    revision: int
    reset: bool
    cells: _containers.RepeatedCompositeFieldContainer[OccupancyCell]
    def __init__(self, cell_size: _Optional[float] = ..., revision: _Optional[int] = ..., reset: _Optional[bool] = ..., cells: _Optional[_Iterable[_Union[OccupancyCell, _Mapping]]] = ...) -> None: ...

class DeviceInfo(_message.Message):
    __slots__ = ("id", "device_type", "timestamp", "transform3", "lidar3", "transform2", "lidar2", "raw_lidar2_scan", "raw_point_cloud")
    ID_FIELD_NUMBER: _ClassVar[int]
    DEVICE_TYPE_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM3_FIELD_NUMBER: _ClassVar[int]
    LIDAR3_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM2_FIELD_NUMBER: _ClassVar[int]
    LIDAR2_FIELD_NUMBER: _ClassVar[int]
    RAW_LIDAR2_SCAN_FIELD_NUMBER: _ClassVar[int]
    RAW_POINT_CLOUD_FIELD_NUMBER: _ClassVar[int]
    id: str
    device_type: DeviceType
    timestamp: float
    transform3: Transform3
    lidar3: Lidar3
    transform2: Transform2
    lidar2: Lidar2
    raw_lidar2_scan: RawLidar2Scan
    raw_point_cloud: RawPointCloud
    def __init__(self, id: _Optional[str] = ..., device_type: _Optional[_Union[DeviceType, str]] = ..., timestamp: _Optional[float] = ..., transform3: _Optional[_Union[Transform3, _Mapping]] = ..., lidar3: _Optional[_Union[Lidar3, _Mapping]] = ..., transform2: _Optional[_Union[Transform2, _Mapping]] = ..., lidar2: _Optional[_Union[Lidar2, _Mapping]] = ..., raw_lidar2_scan: _Optional[_Union[RawLidar2Scan, _Mapping]] = ..., raw_point_cloud: _Optional[_Union[RawPointCloud, _Mapping]] = ...) -> None: ...

class AllDeviceInfo(_message.Message):
    __slots__ = ("devices",)
    DEVICES_FIELD_NUMBER: _ClassVar[int]
    devices: _containers.RepeatedCompositeFieldContainer[DeviceInfo]
    def __init__(self, devices: _Optional[_Iterable[_Union[DeviceInfo, _Mapping]]] = ...) -> None: ...

class Response(_message.Message):
    __slots__ = ("req_uuid", "ok", "err_id_not_found", "err_not_supported", "err_cannot_execute", "err_failed_execute", "all_device_info", "device_info", "lidar_data", "room_bounds", "occupancy_grid")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    OK_FIELD_NUMBER: _ClassVar[int]
    ERR_ID_NOT_FOUND_FIELD_NUMBER: _ClassVar[int]
    ERR_NOT_SUPPORTED_FIELD_NUMBER: _ClassVar[int]
    ERR_CANNOT_EXECUTE_FIELD_NUMBER: _ClassVar[int]
    ERR_FAILED_EXECUTE_FIELD_NUMBER: _ClassVar[int]
    ALL_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    ROOM_BOUNDS_FIELD_NUMBER: _ClassVar[int]
    OCCUPANCY_GRID_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    ok: _empty_pb2.Empty
    err_id_not_found: _empty_pb2.Empty
    err_not_supported: _empty_pb2.Empty
    err_cannot_execute: _empty_pb2.Empty
    err_failed_execute: _empty_pb2.Empty
    all_device_info: AllDeviceInfo
    device_info: DeviceInfo
    lidar_data: Lidar2
    room_bounds: RoomBounds
    occupancy_grid: OccupancyGrid
    def __init__(self, req_uuid: _Optional[str] = ..., ok: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_id_not_found: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_not_supported: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_cannot_execute: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., err_failed_execute: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., all_device_info: _Optional[_Union[AllDeviceInfo, _Mapping]] = ..., device_info: _Optional[_Union[DeviceInfo, _Mapping]] = ..., lidar_data: _Optional[_Union[Lidar2, _Mapping]] = ..., room_bounds: _Optional[_Union[RoomBounds, _Mapping]] = ..., occupancy_grid: _Optional[_Union[OccupancyGrid, _Mapping]] = ...) -> None: ...

class Command(_message.Message):
    __slots__ = ("target_id", "transform3", "transform2")
    TARGET_ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM3_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM2_FIELD_NUMBER: _ClassVar[int]
    target_id: str
    transform3: Transform3
    transform2: Transform2
    def __init__(self, target_id: _Optional[str] = ..., transform3: _Optional[_Union[Transform3, _Mapping]] = ..., transform2: _Optional[_Union[Transform2, _Mapping]] = ...) -> None: ...

class OccupancyGridReq(_message.Message):
    __slots__ = ("device_type", "since_revision")
    DEVICE_TYPE_FIELD_NUMBER: _ClassVar[int]
    SINCE_REVISION_FIELD_NUMBER: _ClassVar[int]
    device_type: DeviceType
    since_revision: int
    def __init__(self, device_type: _Optional[_Union[DeviceType, str]] = ..., since_revision: _Optional[int] = ...) -> None: ...

class ServerRequest(_message.Message):
    __slots__ = ("req_uuid", "post_device_info", "post_heartbeat", "fetch_all_device_info", "fetch_single_device_info", "send_command", "fetch_lidar_data", "clear_accumulated_scan", "fetch_room_bounds", "fetch_occupancy_grid")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    POST_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_HEARTBEAT_FIELD_NUMBER: _ClassVar[int]
    FETCH_ALL_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    FETCH_SINGLE_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    SEND_COMMAND_FIELD_NUMBER: _ClassVar[int]
    FETCH_LIDAR_DATA_FIELD_NUMBER: _ClassVar[int]
    CLEAR_ACCUMULATED_SCAN_FIELD_NUMBER: _ClassVar[int]
    FETCH_ROOM_BOUNDS_FIELD_NUMBER: _ClassVar[int]
    FETCH_OCCUPANCY_GRID_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    post_device_info: DeviceInfo
    post_heartbeat: str
    fetch_all_device_info: _empty_pb2.Empty
    fetch_single_device_info: str
    send_command: Command
    fetch_lidar_data: _empty_pb2.Empty
    clear_accumulated_scan: _empty_pb2.Empty
    fetch_room_bounds: _empty_pb2.Empty
    fetch_occupancy_grid: OccupancyGridReq
    def __init__(self, req_uuid: _Optional[str] = ..., post_device_info: _Optional[_Union[DeviceInfo, _Mapping]] = ..., post_heartbeat: _Optional[str] = ..., fetch_all_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., fetch_single_device_info: _Optional[str] = ..., send_command: _Optional[_Union[Command, _Mapping]] = ..., fetch_lidar_data: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., clear_accumulated_scan: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., fetch_room_bounds: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., fetch_occupancy_grid: _Optional[_Union[OccupancyGridReq, _Mapping]] = ...) -> None: ...

class ClientRequest(_message.Message):
    __slots__ = ("req_uuid", "fetch_device_info", "post_command")
    REQ_UUID_FIELD_NUMBER: _ClassVar[int]
    FETCH_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    POST_COMMAND_FIELD_NUMBER: _ClassVar[int]
    req_uuid: str
    fetch_device_info: _empty_pb2.Empty
    post_command: Command
    def __init__(self, req_uuid: _Optional[str] = ..., fetch_device_info: _Optional[_Union[_empty_pb2.Empty, _Mapping]] = ..., post_command: _Optional[_Union[Command, _Mapping]] = ...) -> None: ...
