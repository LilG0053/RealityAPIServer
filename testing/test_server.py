"""Request handling in server.py, through serialized ServerRequests as a websocket client sends them."""

import json
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from google.protobuf.empty_pb2 import Empty

import server
from map.map import Map
from map.occupancy import OccupancyGrid
from map.tunable import list_tunables
from map.wallgrid import WallGrid
from realityapi_pb2 import (
    Command,
    DeviceInfo,
    DeviceType,
    OccupancyGridReq,
    Response,
    ServerRequest,
    Transform2,
    Vector2,
)


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.original_map = server.server_map
        server.server_map = Map()
        room = server.server_map.room
        # A grid with no file, so nothing here can write to assets/.
        self.occupancy = OccupancyGrid(room)
        server.server_map.wall_grids[DeviceType.TURTLE] = WallGrid(room, self.occupancy)

    def tearDown(self):
        server.server_map = self.original_map

    @staticmethod
    def dispatch(request: ServerRequest) -> Response:
        """Serialize first, just as a websocket client does, then dispatch it."""
        request.req_uuid = request.req_uuid or "test-request"
        received = ServerRequest()
        received.ParseFromString(request.SerializeToString())
        response = server.handle_request(received)
        decoded = Response()
        decoded.ParseFromString(response.SerializeToString())
        return decoded

    def test_reply_echoes_the_request_uuid(self):
        reply = self.dispatch(ServerRequest(req_uuid="abc", fetch_room_bounds=Empty()))
        self.assertEqual(reply.req_uuid, "abc")

    def test_empty_request_is_not_supported(self):
        self.assertTrue(self.dispatch(ServerRequest()).HasField("err_not_supported"))

    def test_handler_errors_are_reported_not_raised(self):
        original = server.HANDLERS["fetch_room_bounds"]
        server.HANDLERS["fetch_room_bounds"] = lambda _: 1 / 0
        try:
            reply = self.dispatch(ServerRequest(fetch_room_bounds=Empty()))
        finally:
            server.HANDLERS["fetch_room_bounds"] = original
        self.assertTrue(reply.HasField("err_failed_execute"))

    def test_room_bounds_are_shared(self):
        reply = self.dispatch(ServerRequest(fetch_room_bounds=Empty()))
        self.assertEqual(reply.room_bounds, server.server_map.room.bounds())
        self.assertAlmostEqual(reply.room_bounds.origin.x, 0.1, places=5)
        self.assertAlmostEqual(reply.room_bounds.origin.y, 2.0, places=5)

    def test_occupancy_grid_per_device_type(self):
        self.occupancy.update(Transform2(position=Vector2(x=2.0, y=5.0)), np.array([[2.05, 6.55]]))
        first = self.dispatch(ServerRequest(fetch_occupancy_grid=OccupancyGridReq(device_type=DeviceType.TURTLE)))
        self.assertTrue(first.occupancy_grid.reset)
        self.assertGreater(len(first.occupancy_grid.cells), 0)

        since = OccupancyGridReq(device_type=DeviceType.TURTLE, since_revision=first.occupancy_grid.revision)
        unchanged = self.dispatch(ServerRequest(fetch_occupancy_grid=since))
        self.assertFalse(unchanged.occupancy_grid.reset)
        self.assertEqual(list(unchanged.occupancy_grid.cells), [])

        dog = self.dispatch(ServerRequest(fetch_occupancy_grid=OccupancyGridReq(device_type=DeviceType.DOG)))
        self.assertTrue(dog.HasField("err_not_supported"))

    def test_unknown_devices(self):
        self.assertTrue(self.dispatch(ServerRequest(post_heartbeat="nobody")).HasField("err_id_not_found"))
        self.assertTrue(self.dispatch(ServerRequest(fetch_single_device_info="nobody")).HasField("err_id_not_found"))
        self.assertTrue(self.dispatch(ServerRequest(send_command=Command(target_id="nobody"))).HasField("err_id_not_found"))
        # A device type the server has no class for cannot register.
        vr = DeviceInfo(id="headset", device_type=DeviceType.VR)
        self.assertTrue(self.dispatch(ServerRequest(post_device_info=vr)).HasField("err_not_supported"))

    def test_registered_device_heartbeat_command_and_timeout(self):
        dog = DeviceInfo(id="dog", device_type=DeviceType.DOG)
        self.assertTrue(self.dispatch(ServerRequest(post_device_info=dog)).HasField("ok"))
        self.assertTrue(self.dispatch(ServerRequest(post_heartbeat="dog")).HasField("ok"))
        # Commands are not forwarded to devices yet.
        self.assertTrue(self.dispatch(ServerRequest(send_command=Command(target_id="dog"))).HasField("err_not_supported"))

        devices = self.dispatch(ServerRequest(fetch_all_device_info=Empty())).all_device_info.devices
        self.assertEqual([d.id for d in devices], ["dog"])

        # Quiet for longer than the timeout: dropped.
        server.server_map._devices["dog"].timestamp = time.time() - server.TIMEOUT - 1
        server.server_map.check_all(server.TIMEOUT)
        self.assertTrue(self.dispatch(ServerRequest(post_heartbeat="dog")).HasField("err_id_not_found"))

    def test_clear_accumulated_scan_resets_the_occupancy_grid(self):
        self.occupancy.update(Transform2(position=Vector2(x=2.0, y=5.0)), np.array([[2.05, 6.55]]))
        request = OccupancyGridReq(device_type=DeviceType.TURTLE)
        revision = self.dispatch(ServerRequest(fetch_occupancy_grid=request)).occupancy_grid.revision
        self.assertTrue(self.dispatch(ServerRequest(clear_accumulated_scan=Empty())).HasField("ok"))
        request.since_revision = revision
        cleared = self.dispatch(ServerRequest(fetch_occupancy_grid=request)).occupancy_grid
        self.assertTrue(cleared.reset)
        self.assertEqual(list(cleared.cells), [])


class DashboardControlTest(unittest.TestCase):
    """The JSON text frames the dashboard uses for its sliders."""

    def control(self, request):
        return server.handle_text_control(json.dumps(request))

    def test_fetch_tunables(self):
        reply = self.control({"id": "1", "action": "fetch_tunables"})
        self.assertTrue(reply["ok"])
        self.assertEqual(reply["id"], "1")
        names = {parameter["name"] for parameter in reply["parameters"]}
        self.assertEqual(names, {parameter["name"] for parameter in list_tunables()})
        self.assertIn("WallGrid.OCCUPANCY_WEIGHT", names)

    def test_set_tunable(self):
        original = WallGrid.OCCUPANCY_WEIGHT
        try:
            reply = self.control({"id": "2", "action": "set_tunable", "name": "WallGrid.OCCUPANCY_WEIGHT", "value": 0.25})
            self.assertTrue(reply["ok"])
            self.assertAlmostEqual(WallGrid.OCCUPANCY_WEIGHT, 0.25)
        finally:
            WallGrid.OCCUPANCY_WEIGHT = original

    def test_bad_requests(self):
        self.assertFalse(self.control({"id": "3", "action": "nonsense"})["ok"])
        self.assertFalse(self.control({"id": "4", "action": "set_tunable", "name": "Nope.NOPE", "value": 1})["ok"])
        self.assertFalse(server.handle_text_control("not json")["ok"])


if __name__ == "__main__":
    unittest.main()
