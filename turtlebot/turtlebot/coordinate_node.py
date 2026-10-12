# FOR TESTING W/ KEYBOARD: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
from time import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from websockets.sync.client import connect
from math import asin, atan2, copysign, pi, isclose


from init_turtlebot import create_turtlebot
from realityapi_pb2 import Packet, Vector3

SERVER_URI = "ws://10.89.51.240:65432"

# The server drops a device after 2 seconds of silence.
HEARTBEAT_INTERVAL = 1.0

# Tolerance values so noise isn't treated as movement
POSITION_TOLERANCE = 0.01
ROTATION_TOLERANCE = 0.01

class CoordinateNode(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        self.robot = create_turtlebot()
        self.ws = connect(SERVER_URI)
        self.send_packet(self.robot.get_pose_packet())

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_update_callback,
            qos_profile_sensor_data
        )
        self.create_timer(HEARTBEAT_INTERVAL, self.heartbeat_callback)

    def pose_update_callback(self, data: Odometry):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        new_pos = Vector3(x=position.x, y=position.z, z=position.y)
        new_rot = self.quaternion_to_euler(orientation)

        # if the position & rotation is same, don't send packet
        if (self.pose_unchanged(new_pos, new_rot, self.robot.pos, self.robot.rot)):
            return

        self.robot.update_position(new_pos)
        self.robot.update_rotation(new_rot)
        self.robot.last_heartbeat = time()
        self.send_packet(self.robot.get_pose_packet())

    # helper method that converts ROS2 Quaternion to Vector3 rotation
    def quaternion_to_euler(orientation):
        # Roll: rotation around the X axis.
        roll = atan2(
            2 * (orientation.w * orientation.x + orientation.y * orientation.z),
            1 - 2 * (orientation.x ** 2 + orientation.y ** 2)
        )

        # Pitch: rotation around the Y axis.
        pitch_input = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        pitch = copysign(pi / 2, pitch_input) if abs(pitch_input) >= 1 else asin(pitch_input)

        # Yaw: rotation around the Z axis.
        yaw = atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        )

        return Vector3(x=roll, y=pitch, z=yaw)

    def pose_unchanged(new_pos, new_rot, prev_pos, prev_rot):
        position_unchanged = (
            isclose(new_pos.x, prev_pos.x, rel_tol=0.0, abs_tol=POSITION_TOLERANCE)
            and isclose(new_pos.y, prev_pos.y, rel_tol=0.0, abs_tol=POSITION_TOLERANCE)
            and isclose(new_pos.z, prev_pos.z, rel_tol=0.0, abs_tol=POSITION_TOLERANCE)
        )

        rotation_unchanged = (
            isclose(new_rot.x, prev_rot.x, rel_tol=0.0, abs_tol=ROTATION_TOLERANCE)
            and isclose(new_rot.y, prev_rot.y, rel_tol=0.0, abs_tol=ROTATION_TOLERANCE)
            and isclose(new_rot.z, prev_rot.z, rel_tol=0.0, abs_tol=ROTATION_TOLERANCE)
        )

        return position_unchanged and rotation_unchanged

    def heartbeat_callback(self):
        self.robot.last_heartbeat = time()
        self.send_packet(self.robot.get_heartbeat_packet())

    def send_packet(self, pkt: Packet):
        self.ws.send(pkt.SerializeToString())
        reply = Packet()
        reply.ParseFromString(self.ws.recv())
        self.get_logger().info(f'>>> {self.describe(pkt)}')
        self.get_logger().info(f'<<< {self.describe(reply)}')

    # console logs position, rotation
    def describe(self, pkt: Packet) -> str:
        if pkt.HasField("heartbeat"):
            return f'heartbeat {pkt.heartbeat.current_time:.3f} id {pkt.id}'
        p = pkt.position
        r = pkt.rotation

        return f'Pos: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) Rot: ({r.z}) id {pkt.id}'

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
