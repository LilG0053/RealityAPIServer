# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
import time
from uuid import uuid4
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect

from init_turtlebot import create_turtlebot
from realityapi_pb2 import Response, ServerRequest, Vector3, RawLidarScan, Transform
from server import SERVER_COORDS

SERVER_URI = "ws://10.89.56.72:65432"

# The server drops a device after 2 seconds of silence.
HEARTBEAT_INTERVAL = 1.0

def interpolate_pose(before, after, scan_time):
        t0, pose0 = before
        t1, pose1 = after
        fraction = (scan_time - t0) / (t1 - t0)

        x = pose0.position.x + fraction * (pose1.position.x - pose0.position.x)
        y = pose0.position.y + fraction * (pose1.position.y - pose0.position.y)

        # Take the shortest path across the -π / +π boundary.
        yaw_change = math.atan2(
            math.sin(pose1.rotation.z - pose0.rotation.z),
            math.cos(pose1.rotation.z - pose0.rotation.z),
        )
        yaw = pose0.rotation.z + fraction * yaw_change
        return x, y, yaw

def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

class CoordinateNode(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        self.robot = create_turtlebot()
        self.ws = connect(SERVER_URI)
        self.post_device_info()

        self.has_odometry = False

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.position_update_callback,
            qos_profile_sensor_data
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_update_callback,
            qos_profile_sensor_data
        )

        self.create_timer(HEARTBEAT_INTERVAL, self.heartbeat_callback)
        self.scan_queue = deque()
        self.odom_queue = deque()

    def send_scan_transform(self, pose: Transform, curOdomTime):
        if self.odom_queue and curOdomTime <= self.odom_queue[-1][0]:
            self.get_logger().warning("Ignoring out-of-order odometry")
            return

        self.odom_queue.append((curOdomTime, pose))

        while self.scan_queue and len(self.odom_queue) >= 2:
            scan_time, scan = self.scan_queue[0]  # Look first; don't remove it yet.

            if scan_time < self.odom_queue[0][0]:
                self.scan_queue.popleft()
                self.get_logger().warning("Dropping scan older than buffered odometry")
                continue

            if scan_time > self.odom_queue[-1][0]:
                break  # Need a later odometry measurement; keep the scan queued.

            # Move the oldest odometry forward until the first two samples
            # surround this scan. Keep both samples for the next queued scan.
            while len(self.odom_queue) > 2 and self.odom_queue[1][0] < scan_time:
                self.odom_queue.popleft()

            before = self.odom_queue[0]
            after = self.odom_queue[1]
            x, y, yaw = interpolate_pose(before, after, scan_time)

            estimated_pose = Transform(
                position=Vector3(x=x, y=y, z=before[1].position.z),
                rotation=Vector3(z=yaw),
            )
            self.send_scan(scan, estimated_pose)
            self.scan_queue.popleft()

        # With no pending scan, only the latest two odometry samples are useful.
        if not self.scan_queue:
            while len(self.odom_queue) > 2:
                self.odom_queue.popleft()

    def position_update_callback(self, data: Odometry):
        self.has_odometry = True
        position = data.pose.pose.position
        pos_vec = Vector3(x=position.x, y=position.y, z=position.z)

        # Yaw about ROS +z from the orientation quaternion.
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rot_vec = Vector3(x=0.0, y=0.0, z=yaw)

        # HANDLES PROCESSING SCAN WITH ACCURATE ESTIMATED ODOM
        pose = Transform(position=pos_vec, rotation=rot_vec)
        curOdomTime = stamp_seconds(data.header.stamp)
        self.send_scan_transform(pose, curOdomTime)

        if pos_vec == self.robot.transform.position and rot_vec == self.robot.transform.rotation:
            return

        self.robot.transform.position.CopyFrom(pos_vec)
        self.robot.transform.rotation.CopyFrom(rot_vec)
        self.post_device_info(log=True)

    def lidar_update_callback(self, data: LaserScan):
        if not self.has_odometry:
            return
        scan_timestamp = stamp_seconds(data.header.stamp)

        if (self.scan_queue and scan_timestamp < self.scan_queue[-1][0]):
            self.get_logger().warning("Dropping scan that is outdated")
            return

        self.scan_queue.append((stamp_seconds(data.header.stamp), data))

    def send_scan(self, ros_data, estimated_robot_pos):
        raw_scan = RawLidarScan(
            id = self.robot.id,
            transform = estimated_robot_pos,
            ranges = ros_data.ranges,
            range_min = ros_data.range_min,
            range_max = ros_data.range_max,
            angle_min = ros_data.angle_min,
            angle_increment = ros_data.angle_increment
        )
        self.send_request(ServerRequest(post_raw_lidar_scan=raw_scan))

    def heartbeat_callback(self):
        self.send_request(ServerRequest(post_heartbeat=self.robot.id))

    def post_device_info(self, log=False):
        self.robot.timestamp = time.time()
        # Odometry updates reuse the last scan already stored on the server.
        device_info = self.robot.get_device_info(
            target_coords=SERVER_COORDS,
            include_lidar=False,
        )
        self.send_request(
            ServerRequest(post_device_info=device_info),
            log=log,
        )

    def send_request(self, req: ServerRequest, log=False, include_lidar=False) -> Response:
        req.req_uuid = str(uuid4())
        started = time.perf_counter()
        self.ws.send(req.SerializeToString())
        reply = Response()
        reply.ParseFromString(self.ws.recv())
        latency_ms = (time.perf_counter() - started) * 1000
        if log:
            self.get_logger().info(f'>>> {self.describe(req, include_lidar=include_lidar)}')
            self.get_logger().info(f'<<< {reply.WhichOneof("data")} | {latency_ms:.1f} ms')
        if reply.req_uuid != req.req_uuid:
            self.get_logger().warning(
                f'response {reply.req_uuid} does not match request {req.req_uuid}')
        return reply

    def describe(self, req: ServerRequest, include_lidar=False) -> str:
        if req.HasField("post_heartbeat"):
            return f'heartbeat id {req.post_heartbeat}'
        info = req.post_device_info
        if include_lidar:
            p = info.transform.position
            return (
                f'lidar {len(info.lidar.points)} points '
                f'at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) id {info.id}'
            )
        p = info.transform.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) id {info.id}'

    def destroy_node(self):
        self.ws.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
