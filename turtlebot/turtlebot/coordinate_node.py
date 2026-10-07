# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
from time import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect

from init_turtlebot import create_turtlebot
from realityapi_pb2 import Packet, Vector3

SERVER_URI = "ws://127.0.0.1:65432"

# The server drops a device after 2 seconds of silence.
HEARTBEAT_INTERVAL = 1.0


class CoordinateNode(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        self.robot = create_turtlebot()
        self.ws = connect(SERVER_URI)
        self.send_packet(self.robot.get_position_packet())

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.position_update_callback,
            qos_profile_sensor_data
        )
        self.create_timer(HEARTBEAT_INTERVAL, self.heartbeat_callback)

    def position_update_callback(self, data: Odometry):
        position = data.pose.pose.position
        new_pos = Vector3(x=position.x, y=position.y, z=position.z)
        if (new_pos.x == self.robot.pos.x
                and new_pos.y == self.robot.pos.y
                and new_pos.z == self.robot.pos.z):
            return

        self.robot.update_position(new_pos)
        self.robot.last_heartbeat = time()
        self.send_packet(self.robot.get_position_packet())

    def heartbeat_callback(self):
        self.robot.last_heartbeat = time()
        self.send_packet(self.robot.get_heartbeat_packet())

    def send_packet(self, pkt: Packet):
        self.ws.send(pkt.SerializeToString())
        reply = Packet()
        reply.ParseFromString(self.ws.recv())
        self.get_logger().info(f'>>> {self.describe(pkt)}')
        self.get_logger().info(f'<<< {self.describe(reply)}')

    def describe(self, pkt: Packet) -> str:
        if pkt.HasField("heartbeat"):
            return f'heartbeat {pkt.heartbeat.current_time:.3f} id {pkt.id}'
        p = pkt.position
        return f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) id {pkt.id}'

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
