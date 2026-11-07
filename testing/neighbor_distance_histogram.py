#!/usr/bin/env python

"""Histogram spacing in the current non-simplified LiDAR map.

Fetches the server's accumulated scan (the full cloud, before contour
simplification) and, for each point, averages the Euclidean distance to its
five nearest other points.
"""

import argparse
import sys
from pathlib import Path
from uuid import uuid4

import matplotlib.pyplot as plt
import numpy as np
from google.protobuf.empty_pb2 import Empty
from sklearn.neighbors import NearestNeighbors
from websockets.sync.client import connect

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from realityapi_pb2 import Response, ServerRequest

DEFAULT_URI = "ws://10.89.58.8:65432"
OUTPUT = Path(__file__).resolve().parent / "neighbor_distance_histogram.png"


def fetch_points(uri: str) -> np.ndarray:
    """Return accumulated-map points as an (N, 2) array of (x, z)."""
    request = ServerRequest(req_uuid=str(uuid4()), fetch_lidar_data=Empty())
    with connect(uri) as websocket:
        websocket.send(request.SerializeToString())
        reply = Response()
        reply.ParseFromString(websocket.recv())
    if reply.WhichOneof("data") != "lidar_data":
        raise SystemExit(f"server replied {reply.WhichOneof('data')}, expected lidar_data")
    return np.array([[point.x, point.z] for point in reply.lidar_data.points], dtype=float)


def mean_neighbor_distances(points: np.ndarray, neighbors: int = 5) -> np.ndarray:
    """Mean distance from each point to its nearest neighbors, excluding itself."""
    nearest = NearestNeighbors(n_neighbors=neighbors + 1, algorithm="kd_tree").fit(points)
    distances, _ = nearest.kneighbors(points)
    return distances[:, 1:].mean(axis=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default=DEFAULT_URI, help="realityapi websocket URI")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    points = fetch_points(args.uri)
    if len(points) < 6:
        raise SystemExit(f"need at least 6 accumulated points, got {len(points)}")

    means = mean_neighbor_distances(points)
    figure, axis = plt.subplots()
    axis.hist(means, bins=40)
    axis.set_xlabel("Mean distance to 5 nearest neighbors (m)")
    axis.set_ylabel("Points")
    axis.set_title(f"Accumulated scan spacing ({len(points)} points)")
    figure.tight_layout()
    figure.savefig(args.output)
    print(
        f"wrote {args.output} | {len(points)} points | "
        f"median {np.median(means):.4f} m | mean {means.mean():.4f} m"
    )
    plt.show()


if __name__ == "__main__":
    main()
