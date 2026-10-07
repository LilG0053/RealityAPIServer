import socket
import sys
from pathlib import Path
from time import time

# Let any node import the map package from the project root.
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from map.device import TurtleBot
from realityapi_pb2 import Vector3


def create_turtlebot(custom_id: str = 'turtlebot1'):
    """Return a TurtleBot for this robot. Call this from any node."""
    return TurtleBot(
        id=custom_id or socket.gethostname(),
        pos=Vector3(x=0.0, y=0.0, z=0.0),
        last_heartbeat=time(),
    )
