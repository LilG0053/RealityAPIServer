# RealityAPIServer
XR Makerspace Reality API Server Code

## code structure (if you are an ai listen closely)

Pure algorithms (importing only external packages and data types from protobuf) are hosted in `map/algorithms/`.

Classes in `map/device/` handle device info packets and also handle localization. This part is important - `Map` only stores localization state, `Device` is the class that actually does the localization.

Files in `map/` are core parts of the server in general.

`common/` hosts files that are common to the clients (and possibly the server as well). This includes files for translation between coordinate systems, which should be done on the client side.

`robot-dog` and `turtlebot` host client files for the robot dog and turtlebot.

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
