from uuid import UUID
from time import time
from map.device import Device, device_from_info, device_from_dict
from realityapi_pb2 import AllDeviceInfo, DeviceInfo, Lidar, Response, Transform
import json, os
import math
import numpy as np

class Map:
    def __init__(self) -> None:
        self._devices: dict[str, Device] = {}
        self.point_map: dict[tuple[int, int], list[tuple[float, float, float]]] = {}

        self.cell_size = 0.10       # meters: each cell is 10 cm × 10 cm
        self.max_points_per_cell = 5

        # creates occupancy grid, key: (cell_x, cell_z) Value: signed evidence score.
        self.occupancy_grid: dict[tuple[int, int], int] = {}

        self.free_evidence = -1
        self.occupied_evidence = 3

        # Limit how strongly an old observation can dominate future observations.
        self.min_evidence = -20
        self.max_evidence = 20
    
    def world_to_cell(self, x: float, z: float) -> tuple[int, int]:
        cell_x = math.floor(x / self.cell_size)
        cell_z = math.floor(z / self.cell_size)

        return (cell_x, cell_z)
    
    def export_global_points(self, filepath: str):
        """Write the accumulated point map as a JSON array."""
        points = []

        for cell_points in self.point_map.values():
            for x, y, z in cell_points:
                points.append({
                    "x": x,
                    "y": y,
                    "z": z,
                })

        # Write to a temporary file first, like export_to_json() does.
        tmp_path = filepath + ".tmp"

        with open(tmp_path, "w") as file:
            json.dump(points, file)

        os.replace(tmp_path, filepath)

    def export_occupancy_grid(self, filepath: str):
        """Export observed cells; absent cells remain unknown."""
        data = {
            "cell_size": self.cell_size,
            "cells": [
                {"x": x,
                    "z": z,
                    "status": 1 if score > 0 else 0,
                } 
                for (x, z), score in self.occupancy_grid.items()
            ],
        }
        tmp_path = filepath + ".tmp"
        with open(tmp_path, "w") as file:
            json.dump(data, file)
        os.replace(tmp_path, filepath)

    def cells_on_ray(self, start: tuple[int, int], end: tuple[int, int],) -> list[tuple[int, int]]:
        """From starting cell (robot) to ending cell (lidar point), 
            return list of all cells in between (free space)
            Uses Breshnham's grid-line algorithm"""
        x, z = start
        end_x, end_z = end

        dx = abs(end_x - x)
        dz = abs(end_z - z)

        step_x = 1 if x < end_x else -1
        step_z = 1 if z < end_z else -1

        error = dx - dz
        cells = []

        while True:
            cells.append((x, z))

            if x == end_x and z == end_z:
                break

            doubled_error = 2 * error

            if doubled_error > -dz:
                error -= dz
                x += step_x

            if doubled_error < dx:
                error += dx
                z += step_z

        return cells
    
    def update_occupancy_grid(self, robot_pos: Transform, corrected_points: np.ndarray,):
        """
        Update the grid from valid LiDAR hits.

        Device transform and corrected_points must all be in the same
        corrected server coordinate frame, before the dashboard offset.
        """

        start_cell = self.world_to_cell(robot_pos.position.x, robot_pos.position.z)

        free_cells = set()
        occupied_cells = set()

        for x, _y, z in corrected_points:
            end_cell = self.world_to_cell(float(x), float(z))
            ray_cells = self.cells_on_ray(start_cell, end_cell)

            # Everything before the endpoint is free-space evidence.
            free_cells.update(ray_cells[:-1])

            # The endpoint is obstacle evidence.
            occupied_cells.add(end_cell)

        # If a cell is both crossed by one ray and hit by another in this
        # scan, treat it as occupied for this update.
        free_cells.difference_update(occupied_cells)

        for cell in free_cells:
            old_score = self.occupancy_grid.get(cell, 0)
            self.occupancy_grid[cell] = max(
                self.min_evidence,
                old_score + self.free_evidence,
            )

        for cell in occupied_cells:
            old_score = self.occupancy_grid.get(cell, 0)
            self.occupancy_grid[cell] = min(
                self.max_evidence,
                old_score + self.occupied_evidence,
            )

    def update_point_map(self, points: np.ndarray):
        """Add corrected, server-coordinate points to the global point map."""
        for x, y, z in points:
            # index for the 2D array corresponding to cell
            cell = self.world_to_cell(x, z) 

            if cell not in self.point_map:
                self.point_map[cell] = []

            if len(self.point_map[cell]) < self.max_points_per_cell:
                self.point_map[cell].append(
                    (float(x), float(y), float(z))
                )

    def add_device(self, device: Device):
        self._devices[device.id] = device
        print(f"Added device: {device} to dict")

    def exists(self, id: str) -> bool:
        return id in self._devices
    
    def get_device_info(self, id: str) -> Response:
        if not id in self._devices:
            print(f"Device with id {id} does not exist")
            return Response(err_id_not_found={})

        dev = self._devices[id]
        return Response(device_info=dev.get_device_info())

    def get_all_device_info(self) -> Response:
        devices = [dev.get_device_info() for dev in self._devices.values()]
        return Response(all_device_info=AllDeviceInfo(devices=devices))

    def del_device(self, id: str) -> Device:
        return self._devices.pop(id)

    def update_last_time(self, id: str, new_time: float):
        self._devices[id].timestamp = new_time

    def heartbeat_check(self, id: str, timeout: float) -> bool:
        """Drop a device that has not been heard from within timeout seconds."""
        device = self._devices[id]

        if device.timestamp is not None and time() - device.timestamp >= timeout:
            print(f"Device {id} is inactive")
            self.del_device(id)
            return True

        return False

    def check_all(self, timeout: float):
        dropped = False
        for id in list(self._devices):
            dropped = self.heartbeat_check(id, timeout) or dropped

        # Re-export so the dashboard stops drawing a device that timed out.
        if dropped:
            self.export_to_json("dashboard/updated_pos.json")

    def get_lidar_data(self) -> Response:
        points = []
        for device in self._devices.values():
            if hasattr(device, "lidar"):
                points.extend(device.lidar.points)
        return Response(lidar_data=Lidar(points=points))

    # function that defines what values are sent to updated_pos.json
    def export_to_json(self, filepath: str):
        devices_data = []

        for device in self._devices.values():
            print(f"{device}")
            devices_data.append(device.to_dict())

        # Write to a temp file, then swap it in, so the dashboard never
        # reads a half-written file.
        tmp_path = filepath + ".tmp"
        with open(tmp_path, 'w') as f:
            json.dump(devices_data, f, indent=2)
        os.replace(tmp_path, filepath)

    def load_from_json(self, filepath: str):
        try:
            with open(filepath, 'r') as f:
                devices_data = json.load(f)

            devices_loaded = 0
            for device_data in devices_data:
                device = device_from_dict(device_data)
                if device is None:
                    print(f"Failed to load device {device_data.get('id')} (may be in wrong format)")
                    continue
                self.add_device(device)
                devices_loaded += 1
            print(f"Loaded {devices_loaded} devices from {filepath}")
        except FileNotFoundError:
            print(f"No existing JSON file at {filepath}, starting with empty map")
        except Exception as e:
            print(f"Error loading from JSON: {e}")

    def handle_device_info(self, device_info: DeviceInfo) -> Response:
        if device_info.id in self._devices: # if id of robot exists, set new position
            device = self._devices[device_info.id]
            device.handle_device_info(device_info)
            print(f"Set position for device: {device.id}")
        else: # if robot id does not exist, add it as a device
            device = device_from_info(device_info)
            if device is None:
                return Response(err_not_supported={})
            self.add_device(device)

        # A position report is also proof the device is still alive. Use the
        # server clock: the float32 wire timestamp is only accurate to ~128s.
        device.timestamp = time()

        # exports file to dashboard folder which updates the visual dashboard
        self.export_to_json("dashboard/updated_pos.json")

        return Response(ok={})

    def handle_heartbeat(self, device_id: str) -> Response:
        if not device_id in self._devices:
            return Response(err_id_not_found={})

        self._devices[device_id].heartbeat()

        return Response(ok={})
