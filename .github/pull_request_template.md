## What this changes

<!-- One or two sentences. Link the issue if there is one. -->

## Where the code went

- [ ] New pure math or algorithms are in `map/algorithms/` and import only external packages, protobuf types and `common/`
- [ ] Localization logic lives in `map/device/`, not in `Map` (`Map` only stores state)
- [ ] Coordinate conversions are in `common/` and run on the client side
- [ ] If `realityapi.proto` changed: `realityapi_pb2.py`, `realityapi_pb2.pyi` and `dashboard/gen/` are regenerated
- [ ] New behavior has a test in `testing/`

## AI use

- [ ] I used an AI tool for part of this, and I have read and can explain every line
