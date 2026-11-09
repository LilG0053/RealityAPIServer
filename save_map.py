#!/usr/bin/env python
"""Save the headsets' depth points as a .ply file you can open in CloudCompare or MeshLab.

Run it from the RealityAPIServer folder while server.py is running and a
headset is sending depth (done ALIGN, map sending on):

    python save_map.py                      # collect for 30 s, write map.ply
    python save_map.py --seconds 60 --voxel 0.03 --out room.ply
    python save_map.py --all                # also include TurtleBot lidar points

While it runs, walk around and look at everything you want in the map. It
asks the server for the latest points twice a second and keeps one point per
voxel (a small cube, 5 cm by default), so the file stays small however long
you scan. Coordinates are server coordinates in meters, same as the dashboard.
"""

import argparse
import sys
import time
from pathlib import Path
from uuid import uuid4

from websockets.sync.client import connect

sys.path.insert(0, str(Path(__file__).resolve().parent))
from realityapi_pb2 import DeviceType, Response, ServerRequest  # noqa: E402


def fetch_devices(ws):
    req = ServerRequest(req_uuid=str(uuid4()), fetch_all_device_info={})
    ws.send(req.SerializeToString())
    reply = Response()
    reply.ParseFromString(ws.recv(timeout=5))
    return reply.all_device_info.devices


def write_ply(path: Path, points):
    with open(path, "w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for x, y, z in points:
            f.write(f"{x:.3f} {y:.3f} {z:.3f}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=30, help="how long to collect (default 30)")
    ap.add_argument("--voxel", type=float, default=0.05, help="cube size in meters; one point kept per cube (default 0.05)")
    ap.add_argument("--out", default="map.ply", help="output file (default map.ply)")
    ap.add_argument("--all", action="store_true", help="include every device's points, not just headsets")
    ap.add_argument("--server", default="ws://127.0.0.1:65432")
    args = ap.parse_args()

    cubes = {}                                    # (i, j, k) cube -> one point in it
    end = time.time() + args.seconds
    with connect(args.server) as ws:
        print(f"Collecting for {args.seconds:.0f} s -- look around the room...")
        while time.time() < end:
            for d in fetch_devices(ws):
                if not args.all and d.device_type != DeviceType.AR:
                    continue
                for p in d.lidar.points:
                    key = (int(p.x // args.voxel), int(p.y // args.voxel), int(p.z // args.voxel))
                    cubes.setdefault(key, (p.x, p.y, p.z))
            print(f"\r  {len(cubes)} points so far, {max(0, end - time.time()):4.0f} s left", end="", flush=True)
            time.sleep(0.5)

    print()
    if not cubes:
        sys.exit("No points received. Is a headset aligned and sending? (Its panel's ALIGN tab shows 'Map sending: ON'.)")
    write_ply(Path(args.out), list(cubes.values()))
    print(f"Wrote {len(cubes)} points to {args.out}")


if __name__ == "__main__":
    main()
