import socket
import sys
from pathlib import Path
from time import time

# Let any node import the map package from the project root.
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from map.device import TurtleBot
from realityapi_pb2 import DeviceType, Transform3, Vector3, Lidar


def create_turtlebot(custom_id: str = 'turtlebot1'):
    """Return a TurtleBot for this robot. Call this from any node.

    Its pose stays in the odometry frame. The server places that frame in the
    room by matching the robot's first scan against assets/blockout.json.
    """
    return TurtleBot(
        id=custom_id or socket.gethostname(),
        device_type=DeviceType.TURTLE,
        timestamp=time(),
        transform=Transform3(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Vector3(x=0.0, y=0.0, z=0.0),
        ),
        lidar=Lidar(),
    )
