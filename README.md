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

## running the server

When in the venv, start the server with `python3 server.py`.

## running the dashboard

Serve the `dashboard` folder (e.g. `cd dashboard && python3 -m http.server`) from the same machine as the server and open it in a browser. It polls the server over `ws://<host>:65432` for device states.

Use **Save scans and maps** to copy the current device poses and latest LiDAR scans (`updated_pos.json`), global point map, and occupancy grid into a timestamped folder under `saved_scans/` on the server. **Clear scans and maps** empties the live files and resets the in-memory maps and ICP localizers while keeping connected devices. If a robot is sending scans, a new map begins immediately. Save first if you want to keep the current data.
