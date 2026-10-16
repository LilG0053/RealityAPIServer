# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
from time import time
from uuid import uuid4

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect

from init_turtlebot import create_turtlebot
from realityapi_pb2 import Response, ServerRequest, Vector3

SERVER_URI = "ws://10.89.51.240:65432"

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
        self.create_timer(HEARTBEAT_INTERVAL, self.heartbeat_callback)

    def position_update_callback(self, data: Odometry):
        position = data.pose.pose.position
        new_pos = Vector3(x=position.x, y=position.z, z=position.y)
        if new_pos == self.robot.transform.position:
            return

        self.robot.transform.position.CopyFrom(new_pos)
        self.post_device_info()

    def heartbeat_callback(self):
        self.send_request(ServerRequest(post_heartbeat=self.robot.id))

    def post_device_info(self):
        self.robot.timestamp = time()
        self.send_request(ServerRequest(post_device_info=self.robot.get_device_info()))

    def send_request(self, req: ServerRequest) -> Response:
        req.req_uuid = str(uuid4())
        self.ws.send(req.SerializeToString())
        reply = Response()
        reply.ParseFromString(self.ws.recv())
        self.get_logger().info(f'>>> {self.describe(req)}')
        self.get_logger().info(f'<<< {reply.WhichOneof("data")}')
        if reply.req_uuid != req.req_uuid:
            self.get_logger().warning(
                f'response {reply.req_uuid} does not match request {req.req_uuid}')
        return reply

    def describe(self, req: ServerRequest) -> str:
        if req.HasField("post_heartbeat"):
            return f'heartbeat id {req.post_heartbeat}'
        p = req.post_device_info.transform.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) id {req.post_device_info.id}'

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
