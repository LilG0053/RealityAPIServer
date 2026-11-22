# RealityAPIServer
XR Makerspace Reality API Server Code

## code structure (if you are an ai listen closely)

Pure algorithms (importing only external packages, data types from protobuf, and `common/`) are hosted in `map/algorithms/`.

Classes in `map/device/` handle device info packets and also handle localization. This part is important - `Map` only stores localization state, `Device` is the class that actually does the localization.

Files in `map/` are core parts of the server in general.

`common/` hosts files that are common to the clients (and possibly the server as well). This includes files for translation between coordinate systems, which should be done on the client side.

`clients/robot-dog` and `clients/turtlebot` host client files for the robot dog and turtlebot.

`testing` holds unit tests.

The protobuf is hosted in `realityapi.proto`. Modify if necessary but try to stick very closely to the structure that it has right now.

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

## running the server

When in the venv, start the server with `python3 server.py`.

## running the dashboard

Serve the `dashboard` folder (e.g. `cd dashboard && python3 -m http.server`) from the same machine as the server and open it in a browser. It polls the server over `ws://<host>:65432` for device states.

## robot localization phases (See this to understand at a high level how the project currently works)

1. **Post.** The robot sends its raw lidar scan in `post_device_info`. Before it has been placed, it sends a scan every few seconds with no pose.
2. **Global localization (branch and bound).** The server searches every position and heading in the room for the best fit of the scan against the `WallGrid` (blockout walls blended with the shared occupancy grid). A match seeds the particle filter. If there is no match, the reply has no pose and the robot retries.
3. **Tracking (AMCL).** Once the robot is placed, each post that shows movement runs a particle filter: it predicts from the odometry change, weights particles with the likelihood field, and resamples. The reported pose is the heaviest particle cluster. If the fit stays poor, or the reported pose jumps, branch and bound runs again.
4. **Mapping.** A scan is placed at the estimated pose and checked against the robot's accumulated map. If it passes, it is merged into the shared `OccupancyGrid`, and `WallGrid` picks up the change on its next refresh. A parked robot only contributes its first few scans.
5. **Handover.** The reply carries the robot's room pose. The robot turns it into an on-device correction (`room = correction ∘ odom`) and reports in room coordinates from then on. The server resets its own correction to identity.
