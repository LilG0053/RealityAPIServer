import socket
import sys
from pathlib import Path

# Let any node import the common package and realityapi_pb2 from the project root.
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.coordinate import SERVER_COORDS, TURTLEBOT_COORDS, convert_transform
from common.transform2 import compose_transform2, invert_transform2, transform2_from_transform3
from realityapi_pb2 import Transform2, Transform3


class TurtleBotPose:
    """On-device coordinate correction from this robot's odometry to the server frame.

    The server localizes the robot (branch and bound, then Monte Carlo) and
    answers fetch_single_device_info with its pose in the room. adopt() turns
    that answer into a correction, so every later pose is sent in server
    coordinates.
    """

    def __init__(self, device_id: str):
        self.id = device_id
        # Latest odometry, converted to server axes (not yet placed in the room).
        self.odom: Transform2 | None = None
        # Odometry at the last post. The server's answer describes this pose.
        self.posted_odom: Transform2 | None = None
        # Server pose = compose_transform2(correction, odometry). None until placed.
        self.correction: Transform2 | None = None

    @property
    def localized(self) -> bool:
        return self.correction is not None

    def set_odometry(self, ros: Transform3) -> Transform2:
        """Take a ROS odometry pose (x north, y west, z up; rotation.z is the yaw about z)."""
        self.odom = transform2_from_transform3(convert_transform(ros, TURTLEBOT_COORDS, SERVER_COORDS))
        return self.odom

    def server_pose(self) -> Transform2:
        """The latest odometry in server coordinates."""
        return compose_transform2(self.correction, self.odom)

    def adopt(self, server_pose: Transform2) -> None:
        """Take on the server's pose for the last post."""
        self.correction = compose_transform2(server_pose, invert_transform2(self.posted_odom))

    def forget(self) -> None:
        """The server no longer has this robot placed; localize again."""
        self.correction = None


def create_turtlebot(custom_id: str = 'turtlebot1') -> TurtleBotPose:
    """Return the pose state for this robot. Call this from any node."""
    return TurtleBotPose(custom_id or socket.gethostname())
