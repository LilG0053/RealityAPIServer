"""Run an isolated synthetic dashboard demo without replacing robot map files.

From the project root: python -m testing.demo_dashboard
"""

import argparse
import asyncio
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
import shutil
import tempfile
import threading

import numpy as np
from websockets.asyncio.server import serve

from map.localization import ScanLocalizer
from map.map import Map
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, DeviceType, Lidar, Response, ServerRequest, Transform, Vector3


ROOT = Path(__file__).resolve().parents[1]


def scene_points():
    """An illustrative room outline and furniture in dashboard x/z coordinates."""
    segments = [((1.0, 1.0), (4.0, 1.0)), ((4.0, 1.0), (4.0, 4.0)),
                ((4.0, 4.0), (1.0, 4.0)), ((1.0, 4.0), (1.0, 1.0)),
                ((1.7, 2.1), (2.7, 2.1)), ((2.7, 2.1), (2.7, 2.8))]
    return np.array([
        np.asarray(start) + fraction * (np.asarray(end) - np.asarray(start))
        for start, end in segments
        for fraction in np.linspace(0, 1, 36, endpoint=False)
    ])


class DashboardDemo:
    def __init__(self, dashboard_path):
        self.dashboard_path = Path(dashboard_path)
        self.world = Map()
        self.localizer = ScanLocalizer()
        self.reference = scene_points()
        self.device = None

    def update(self, stage):
        # This is illustrative input, not physically simulated LiDAR ray casting.
        visible = self.reference[:36 * (stage + 1)]
        error = np.array([0.006 * stage, -0.004 * stage])
        xz = visible + error
        points = np.column_stack((xz[:, 0], np.full(len(xz), 0.08), xz[:, 1]))
        status, aligned = self.localizer.update_correction(points)
        if status not in ('first_scan', 'updated'):
            raise RuntimeError(f'ICP could not update stage {stage + 1}: {status}')
        mean_error = float(np.mean(np.linalg.norm(aligned[:, [0, 2]] - visible, axis=1)))

        # Apply the same correction to the illustrative sensor origin.
        raw_pose = Transform(position=Vector3(x=2.5 + error[0], y=0.08, z=2.5 + error[1]))
        pose = self.localizer.corrected_transform(raw_pose, (0.0, 0.0, 0.0))
        self.world.update_point_map(aligned)
        self.world.update_occupancy_grid(pose, aligned)
        self.world.export_global_points(str(self.dashboard_path / 'global_map.json'))
        self.world.export_occupancy_grid(str(self.dashboard_path / 'occupancy_grid.json'))

        scan = Lidar(points=[Vector3(x=float(x), y=float(y), z=float(z)) for x, y, z in aligned])
        self.device = DeviceInfo(id='demo-robot', device_type=DeviceType.TURTLE,
                                 transform=pose, lidar=scan)
        count = sum(len(cell) for cell in self.world.point_map.values())
        print(f'Stage {stage + 1}/6: ICP={status}, map points={count}, '
              f'occupancy cells={len(self.world.occupancy_grid)}, '
              f'mean known-point error={mean_error:.4f} m', flush=True)

    async def handle_client(self, websocket):
        async for message in websocket:
            if not isinstance(message, bytes):
                continue
            request = ServerRequest()
            try:
                request.ParseFromString(message)
            except Exception:
                continue
            if request.WhichOneof('data') == 'fetch_all_device_info':
                devices = [self.device] if self.device is not None else []
                response = Response(req_uuid=request.req_uuid,
                                    all_device_info=AllDeviceInfo(devices=devices))
            else:
                response = Response(req_uuid=request.req_uuid, err_not_supported={})
            await websocket.send(response.SerializeToString())


async def run_demo(dashboard_path, http_port, interval):
    demo = DashboardDemo(dashboard_path)
    demo.update(0)
    handler = partial(SimpleHTTPRequestHandler, directory=str(dashboard_path))
    http = ThreadingHTTPServer(('127.0.0.1', http_port), handler)
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    try:
        # The current dashboard connects to port 65432. No real server should
        # be using that port while this isolated demonstration is running.
        async with serve(demo.handle_client, '127.0.0.1', 65432):
            thread.start()
            print(f'Open http://localhost:{http_port}/', flush=True)
            print('Synthetic demo only. Press Ctrl+C to stop.', flush=True)
            for stage in range(1, 6):
                await asyncio.sleep(interval)
                demo.update(stage)
            print('All stages complete; dashboard remains available.', flush=True)
            await asyncio.Future()
    finally:
        if thread.is_alive():
            http.shutdown()
            thread.join()
        http.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--http-port', type=int, default=8000)
    parser.add_argument('--interval', type=float, default=5.0)
    args = parser.parse_args()
    if not 1 <= args.http_port <= 65535 or not math.isfinite(args.interval) or args.interval <= 0:
        parser.error('Choose a valid port and a positive finite interval.')
    try:
        with tempfile.TemporaryDirectory(prefix='reality-dashboard-demo-') as tmp:
            dashboard = Path(tmp) / 'dashboard'
            shutil.copytree(ROOT / 'dashboard', dashboard,
                            ignore=shutil.ignore_patterns('*.json', 'node_modules', '__pycache__'))
            asyncio.run(run_demo(dashboard, args.http_port, args.interval))
    except KeyboardInterrupt:
        print('\nDemo stopped.')
    except OSError as exc:
        raise SystemExit(f'Could not start demo: {exc}. Check whether ports {args.http_port} or 65432 are in use.')


if __name__ == '__main__':
    main()
