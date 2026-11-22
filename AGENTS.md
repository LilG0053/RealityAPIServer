# Instructions for AI coding agents (and humans)

Read this before changing code. The import rules below are checked by
`testing/test_structure.py` (rules in `.importlinter`). Run the tests before opening a PR.

## Layers

Code may import from layers below it, never above:

1. `server.py`: websocket handling and request routing only
2. `map/map.py`: `Map` stores localization state. It does not localize.
3. `map/device/`: `Device` subclasses handle device packets and do the localization
4. `map/wallgrid.py`, `map/occupancy.py`, `map/room.py`, `map/tunable.py`: core server data structures
5. `map/algorithms/`: pure algorithms. Import only external packages and protobuf types.
   Never import from `map/`, `common/` or `server`. If an algorithm needs a project
   type, define an interface here (see `wallgridinterface.py`).

`common/` is shared with the clients and must not import from `map/` or `server`.
Coordinate-system conversion belongs in `common/` and runs on the client.

`clients/robot-dog/` and `clients/turtlebot/` hold client code. Tests go in `testing/`.

## Protocol

`realityapi.proto` is the wire format. Change it only when necessary and keep its
current structure. After changing it, regenerate the bindings (see README).

## Before opening a PR

- `python -m pytest testing` passes (this includes the import-layer check in `test_structure.py`)
- Keep PRs small and focused. Do not mix refactors with features.
