# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
# FOR POSE RESET: ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}"
import argparse
import sys
import time
from uuid import uuid4
import math

import rclpy
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect

from init_turtlebot import create_turtlebot
from realityapi_pb2 import DeviceInfo, DeviceType, RawLidar2Scan, Response, ServerRequest, Transform3, Vector3

# Used when no server URI is passed on the command line.
SERVER_URI = "ws://0.0.0.0:65432"

# The server drops a device after 2 seconds of silence.
HEARTBEAT_INTERVAL = 1.0

# Seconds between fetches of this robot's own pose: quickly while the server is
# placing it, then now and then to pick up the server's Monte Carlo refinements.
PLACEMENT_POLL_INTERVAL = 0.5
POSE_REFRESH_INTERVAL = 2.0

# Seconds between localization scans while the server has not placed the robot.
LOCALIZE_RETRY = 1.0


class CoordinateNode(Node):
    """Streams odometry and lidar to the server, which localizes the robot.

    1. Send a raw scan (no pose yet) until the server places the robot with it.
    2. Poll fetch_single_device_info until the server returns a pose.
    3. Turn that pose into an on-device correction (TurtleBotPose.adopt), and
       fetch again now and then to pick up the server's refinements.
    4. Send every pose and scan in server coordinates through that correction.
    """

    def __init__(self, server_uri: str = SERVER_URI):
        super().__init__('coordinate_node')
        self.robot = create_turtlebot()
        self.get_logger().info(f'Connecting to {server_uri}')
        self.ws = connect(server_uri)

        # True once the server has placed the robot, until the pose is fetched.
        self.awaiting_placement = False
        self.last_localize_post = -math.inf
        self.last_fetch = -math.inf

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
        self.create_timer(PLACEMENT_POLL_INTERVAL, self.fetch_pose_callback)
        self.get_logger().info('Waiting for the server to localize this robot')

    def position_update_callback(self, data: Odometry):
        position = data.pose.pose.position

        # Yaw about ROS +z from the orientation quaternion.
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        previous = self.robot.odom
        ros = Transform3(
            position=Vector3(x=position.x, y=position.y, z=position.z),
            rotation=Vector3(z=yaw),
        )
        if self.robot.set_odometry(ros) == previous:
            return
        # Until the server places the robot there is no server pose to send.
        if self.robot.localized:
            self.post_device_info(log=True)

    def lidar_update_callback(self, data: LaserScan):
        if self.robot.odom is None:
            return  # no odometry yet to pair the scan with
        raw_scan = RawLidar2Scan(
            ranges=data.ranges,
            range_min=data.range_min,
            range_max=data.range_max,
            angle_min=data.angle_min,
            angle_increment=data.angle_increment,
        )
        if self.robot.localized:
            self.post_device_info(raw_scan)
            return

        # Not placed yet: send a scan for global localization, then wait for
        # fetch_pose_callback to pick up the result.
        now = time.monotonic()
        if self.awaiting_placement or now - self.last_localize_post < LOCALIZE_RETRY:
            return
        self.last_localize_post = now
        reply = self.post_device_info(raw_scan, log=True)
        if reply.HasField('ok'):
            self.awaiting_placement = True
            self.get_logger().info('Server placed the robot; fetching its pose')
        else:
            self.get_logger().warning(
                f'Server could not place the robot yet ({reply.WhichOneof("data")}); retrying')

    def fetch_pose_callback(self):
        if not (self.awaiting_placement or self.robot.localized):
            return
        now = time.monotonic()
        if self.robot.localized and now - self.last_fetch < POSE_REFRESH_INTERVAL:
            return
        self.last_fetch = now

        reply = self.send_request(ServerRequest(fetch_single_device_info=self.robot.id))
        if reply.HasField('device_info') and reply.device_info.HasField('transform2'):
            first = not self.robot.localized
            self.robot.adopt(reply.device_info.transform2)
            self.awaiting_placement = False
            if first:
                pose = self.robot.server_pose()
                self.get_logger().info(
                    f'Localized at ({pose.position.x:.2f}, {pose.position.y:.2f}), '
                    f'heading {math.degrees(pose.rotation):.1f} deg')
        elif self.robot.localized or reply.HasField('err_id_not_found'):
            # The server dropped or un-placed this robot: localize again.
            self.get_logger().warning(
                f'Server has no pose for this robot ({reply.WhichOneof("data")}); localizing again')
            self.robot.forget()
            self.awaiting_placement = False

    def heartbeat_callback(self):
        self.send_request(ServerRequest(post_heartbeat=self.robot.id))

    def post_device_info(self, raw_scan: RawLidar2Scan | None = None, log=False) -> Response:
        """Send the current pose (once placed) and an optional raw scan."""
        info = DeviceInfo(id=self.robot.id, device_type=DeviceType.TURTLE, timestamp=time.time())
        if self.robot.localized:
            info.transform2.CopyFrom(self.robot.server_pose())
        if raw_scan is not None:
            info.raw_lidar2_scan.CopyFrom(raw_scan)
        # The server's next answer describes the pose at this post.
        self.robot.posted_odom = self.robot.odom
        return self.send_request(ServerRequest(post_device_info=info), log=log)

    def send_request(self, req: ServerRequest, log=False) -> Response:
        req.req_uuid = str(uuid4())
        started = time.perf_counter()
        self.ws.send(req.SerializeToString())
        reply = Response()
        reply.ParseFromString(self.ws.recv())
        latency_ms = (time.perf_counter() - started) * 1000
        if log:
            self.get_logger().info(f'>>> {self.describe(req)}')
            self.get_logger().info(f'<<< {reply.WhichOneof("data")} | {latency_ms:.1f} ms')
        if reply.req_uuid != req.req_uuid:
            self.get_logger().warning(
                f'response {reply.req_uuid} does not match request {req.req_uuid}')
        return reply

    def describe(self, req: ServerRequest) -> str:
        if req.HasField("post_heartbeat"):
            return f'heartbeat id {req.post_heartbeat}'
        if req.HasField("fetch_single_device_info"):
            return f'fetch pose id {req.fetch_single_device_info}'
        info = req.post_device_info
        parts = []
        if info.HasField("transform2"):
            p = info.transform2.position
            parts.append(f'({p.x:.3f}, {p.y:.3f}) heading {info.transform2.rotation:.3f}')
        if info.HasField("raw_lidar2_scan"):
            parts.append(f'scan {len(info.raw_lidar2_scan.ranges)} ranges')
        return f'{" ".join(parts) or "empty"} id {info.id}'

    def destroy_node(self):
        self.ws.close()
        super().destroy_node()

def parse_args(args=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Stream TurtleBot odometry and lidar to the Reality API server.')
    parser.add_argument(
        'server_uri',
        nargs='?',
        default=SERVER_URI,
        help=f'websocket URI of the server (default: {SERVER_URI})',
    )
    # ROS arguments (--ros-args ...) are for rclpy, not this parser.
    return parser.parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])


def main(args=None):
    options = parse_args(args)
    rclpy.init(args=args)
    node = CoordinateNode(options.server_uri)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
