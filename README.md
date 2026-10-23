# RealityAPIServer
XR Makerspace Reality API Server Code

## venv

It is recommended to use a virtual environment for running the server/client. Use `python3 -m venv venv` and then run `source venv/bin/activate`. Then you can pip install stuff easier.

## building protobuf

```
pip install grpcio-tools
python -m grpc_tools.protoc -I. --python_out=. --pyi_out=. realityapi.proto
```

The dashboard uses its own JS bindings in `dashboard/gen/`. Rebuild them after changing the proto:

```
cd dashboard
npm install
npm run build:proto
```

## lidar topic

The native ROS 2 dog client subscribes to the pose topic
`/utlidar/robot_pose` and the Unitree lidar topic
`/utlidar/cloud_deskewed` (`sensor_msgs/msg/PointCloud2`). It decodes the
`x`, `y`, and `z` point fields and sends them in the existing
`DeviceInfo.lidar.points` protobuf field. The dashboard renders the points
relative to the reported robot pose.

Run the native client from a ROS 2 environment where `rclpy`, `geometry_msgs`,
and `sensor_msgs` are sourced:

```
python robot-dog/dog_client.py
```

Dense scans are decimated to at most 20,000 points per update to keep the
WebSocket and browser responsive. The ordered points are then simplified
using 3D Ramer-Douglas-Peucker before protobuf serialization. The default
`epsilon` is `0.05` meters; increase it for fewer points and lower latency.
Configure it without changing source code:

```
REALITYAPI_LIDAR_EPSILON=0.05 python robot-dog/dog_client.py
```

The `epsilon` argument is also available directly through
`lidar_request(message, epsilon=...)` for tests or another client.

## running the server

When in the venv, start the server with `python3 server.py`.

## running the dashboard

Serve the `dashboard` folder (e.g. `cd dashboard && python3 -m http.server`) from the same machine as the server and open it in a browser. It polls the server over `ws://<host>:65432` for device states.
