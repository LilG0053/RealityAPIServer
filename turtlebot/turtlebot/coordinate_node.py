# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
import time
from uuid import uuid4
import math

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect

from init_turtlebot import create_turtlebot
from map.lidar import simplify_lidar_2d
from realityapi_pb2 import Response, ServerRequest, Vector3, Lidar
from server import SERVER_COORDS

SERVER_URI = "ws://0.0.0.0:65432"

# Contour tolerance, in meters, applied to each scan before it is posted.
LIDAR_EPSILON = 0.05

# The server drops a device after 2 seconds of silence.
HEARTBEAT_INTERVAL = 1.0


class CoordinateNode(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        self.robot = create_turtlebot()
        self.ws = connect(SERVER_URI)
        self.post_device_info()

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

    def position_update_callback(self, data: Odometry):
        position = data.pose.pose.position
        pos_vec = Vector3(x=position.x, y=position.y, z=position.z)

        # Yaw about ROS +z from the orientation quaternion.
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rot_vec = Vector3(x=0.0, y=0.0, z=yaw)

        if pos_vec == self.robot.transform.position and rot_vec == self.robot.transform.rotation:
            return

        self.robot.transform.position.CopyFrom(pos_vec)
        self.robot.transform.rotation.CopyFrom(rot_vec)
        self.post_device_info(log=True)

    def lidar_update_callback(self, data: LaserScan):
        range_min = data.range_min
        range_max = data.range_max
        angle_min = data.angle_min
        delta = data.angle_increment
        transform = self.robot.transform
        # Convert each LiDAR range reading into a point in the robot's world-fixed odom frame.
        #
        # Computing the ray angle `theta` in the odom XY plane requires chaining three angles:
        #   1. `(i * delta) + angle_min`:
        #      The angle of beam `i` in the LiDAR sensor's own frame (`rplidar_link`),
        #      measured counterclockwise from the sensor's 0-radian axis.
        #   2. `+ (math.pi / 2)`:
        #       transforms from `rplidar_link` into the robot body frame (`base_link`).
        #      On the TurtleBot 4, the RPLIDAR is physically mounted rotated +90 degrees
        #      counterclockwise (rpy="0 0 pi/2" in turtlebot4.urdf.xacro), so the sensor's
        #      0-radian axis points out the left side (+y) of `base_link` rather than the
        #      front (+x). Adding pi/2 aligns 0 radians with the robot's forward +x axis.
        #   3. `+ transform.rotation.z`:
        #      Transforms from the rotating `base_link` frame into the world-fixed `odom`
        #      frame by adding the robot's current yaw relative to its starting orientation.
        points = []
        for i, r in enumerate(data.ranges):
            if range_min < r < range_max:
                theta = (i * delta) + angle_min + (math.pi / 2) + transform.rotation.z
                points.append(
                    Vector3(
                        x=transform.position.x + (r * math.cos(theta)),
                        y=transform.position.y + (r * math.sin(theta)),
                        z=transform.position.z,
                    )
                )
        self.robot.lidar = simplify_lidar_2d(Lidar(points=points), epsilon=LIDAR_EPSILON)
        self.post_device_info(log=True, include_lidar=True)

    def heartbeat_callback(self):
        self.send_request(ServerRequest(post_heartbeat=self.robot.id))

    def post_device_info(self, log=False, include_lidar=False):
        self.robot.timestamp = time.time()
        # Odometry updates reuse the last scan already stored on the server.
        device_info = self.robot.get_device_info(
            target_coords=SERVER_COORDS,
            include_lidar=include_lidar,
        )
        self.send_request(
            ServerRequest(post_device_info=device_info),
            log=log,
            include_lidar=include_lidar,
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
