import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { create, fromBinary, toBinary } from "@bufbuild/protobuf";
import { DeviceType, ResponseSchema, ServerRequestSchema } from "./gen/realityapi_pb.js";

const sceneContainer = document.getElementById("scene-container");

// Room measurements
// One Three.js unit represents one real-world meter.
// Width is X, height is Y, depth is Z.
const WALL_HEIGHT = 3;
const WALL_THICKNESS = 0.15;

// The blockout has no heights, so obstacles are drawn as tall as the walls.
const OBSTACLE_HEIGHT = WALL_HEIGHT;

// Room corners (in meters), from the server's RoomBounds (assets/blockout.json).
// (0, 0) is the corner furthest from the door. Empty until the server answers.
let roomCorners = [];

// Which device type's occupancy grid the dashboard shows (the room is shared).
// Set from the "Occupancy grid for" picker, or ?device=TURTLE (any DeviceType
// name) in the URL.
const OCCUPANCY_DEVICE_TYPES = Object.keys(DeviceType)
  .filter((name) => Number.isNaN(Number(name)) && name !== "NONE");
let occupancyDeviceType = (() => {
  const named = new URLSearchParams(location.search).get("device")?.toUpperCase();
  return OCCUPANCY_DEVICE_TYPES.includes(named) ? DeviceType[named] : DeviceType.TURTLE;
})();

// Scene, camera, and renderer
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x20232a);

const camera = new THREE.PerspectiveCamera(
  55,
  sceneContainer.clientWidth / sceneContainer.clientHeight,
  0.1,
  1000
);
// Start outside the negative-X/positive-Z corner and look at the origin.
// This changes only the viewer's side of the room; it does not alter the
// measured X/Z coordinates used by the floor, walls, or robot markers.
camera.position.set(-3, 15, 10);
camera.lookAt(0, 0, 0);

// Mirror the full scene so positive sensor Z values match the physical room layout.
scene.scale.z = -1;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(sceneContainer.clientWidth, sceneContainer.clientHeight);
sceneContainer.appendChild(renderer.domElement);

// Camera controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.enableRotate = true;
controls.enablePan = true;
controls.screenSpacePanning = true;
controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
controls.touches.ONE = THREE.TOUCH.ROTATE;
controls.touches.TWO = THREE.TOUCH.DOLLY_PAN;
// These limits keep the camera above the floor and away from a full side view.
controls.minPolarAngle = Math.PI / 6;
controls.maxPolarAngle = Math.PI / 2.3;
controls.minDistance = 2;
controls.maxDistance = 30;

// Room lighting
scene.add(new THREE.HemisphereLight(0xffffff, 0x2b3340, 2));

const directionalLight = new THREE.DirectionalLight(0xffffff, 2);
directionalLight.position.set(5, 10, 5);
scene.add(directionalLight);

// Coordinate axes
// X is red, Y is green, and Z is blue. All axes begin at (0, 0, 0).
scene.add(new THREE.AxesHelper(2));

function createMapLabel(text, color, position, size = 0.45) {
  const canvas = document.createElement("canvas");
  canvas.height = 128;

  let context = canvas.getContext("2d");
  context.font = "bold 72px Arial";
  canvas.width = Math.ceil(context.measureText(text).width + 40);

  // Setting canvas width clears its drawing settings.
  context = canvas.getContext("2d");
  context.fillStyle = color;
  context.font = "bold 72px Arial";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(text, canvas.width / 2, canvas.height / 2);

  const label = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas) })
  );
  label.position.set(...position);
  label.scale.set(size * (canvas.width / canvas.height), size, 1);
  scene.add(label);
}

createMapLabel("X", "#ff5555", [2.3, 0.15, 0]);
createMapLabel("Y", "#65dc65", [0, 2.3, 0]);
createMapLabel("Z", "#6fa8ff", [0, 0.15, 2.3]);

// Room walls
function createWallBetween(start, end) {
  const dx = end.x - start.x;
  const dz = end.z - start.z;
  const length = Math.hypot(dx, dz);

  const wall = new THREE.Mesh(
    new THREE.BoxGeometry(length, WALL_HEIGHT, WALL_THICKNESS),
    new THREE.MeshStandardMaterial({
      color: 0xe8e8e8,
      roughness: 0.8,
      transparent: true,
      opacity: 0.65,
      side: THREE.DoubleSide
    })
  );

  wall.position.set(
    (start.x + end.x) / 2,
    WALL_HEIGHT / 2,
    (start.z + end.z) / 2
  );
  wall.rotation.y = Math.atan2(-dz, dx);
  return wall;
}

// Shape in the X/Z ground plane. Rotating it +90 degrees about X maps the
// shape's y onto Z, the same way for the floor and the obstacles.
function groundShape(points) {
  const shape = new THREE.Shape();
  shape.moveTo(points[0].x, points[0].z);
  for (const point of points.slice(1)) {
    shape.lineTo(point.x, point.z);
  }
  shape.closePath();
  return shape;
}

// Floor, walls, outline, and obstacles all come from the server, so they
// live in one group that is rebuilt whenever the room changes.
const roomGroup = new THREE.Group();
scene.add(roomGroup);
let roomKey = "";

// RoomBounds polygons are Vector2s on the X/Z ground plane: Vector2.y is Z.
function toGroundPoints(polygon) {
  return (polygon?.vertices ?? []).map((vertex) => ({ x: vertex.x, z: vertex.y }));
}

// Where server (0, 0) is drawn. Poses, scans, and the accumulated map arrive in
// server coordinates and are shifted by this before drawing.
let serverOrigin = { x: 0, z: 0 };

function buildRoom(bounds) {
  serverOrigin = { x: bounds.origin?.x ?? 0, z: bounds.origin?.y ?? 0 };
  const corners = toGroundPoints(bounds.room);
  const obstacles = bounds.obstacles.map(toGroundPoints).filter((points) => points.length >= 3);
  const key = JSON.stringify([corners, obstacles]);
  if (corners.length < 3 || key === roomKey) return;
  roomKey = key;
  roomCorners = corners;

  roomGroup.traverse((object) => {
    object.geometry?.dispose();
    object.material?.dispose();
  });
  roomGroup.clear();

  // Room floor
  // ShapeGeometry fills the exact outline from roomCorners.
  const floor = new THREE.Mesh(
    new THREE.ShapeGeometry(groundShape(roomCorners)),
    new THREE.MeshBasicMaterial({
      color: 0x68717d,
      side: THREE.DoubleSide
    })
  );
  floor.rotation.x = Math.PI / 2;
  roomGroup.add(floor);

  // Room walls
  for (let index = 0; index < roomCorners.length; index += 1) {
    const start = roomCorners[index];
    const end = roomCorners[(index + 1) % roomCorners.length];
    roomGroup.add(createWallBetween(start, end));
  }

  const roomOutline = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(
      roomCorners.map((corner) => new THREE.Vector3(corner.x, 0.02, corner.z))
    ),
    new THREE.LineBasicMaterial({ color: 0xffffff })
  );
  roomGroup.add(roomOutline);

  // Obstacles (pillars, cabinets): solid blocks standing on their footprints.
  // The extrusion runs along the shape's +Z, which the rotation turns into
  // world -Y, so the block is lifted by its height to stand on the floor.
  for (const points of obstacles) {
    const block = new THREE.Mesh(
      new THREE.ExtrudeGeometry(groundShape(points), { depth: OBSTACLE_HEIGHT, bevelEnabled: false }),
      new THREE.MeshStandardMaterial({
        color: 0xb0b8c4,
        roughness: 0.8,
        transparent: true,
        opacity: 0.8
      })
    );
    block.rotation.x = Math.PI / 2;
    block.position.y = OBSTACLE_HEIGHT;
    roomGroup.add(block);

    const outline = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(
        points.map((point) => new THREE.Vector3(point.x, 0.02, point.z))
      ),
      new THREE.LineBasicMaterial({ color: 0xffffff })
    );
    roomGroup.add(outline);
  }

  // The occupancy lattice covers the room, so rebuild it for the new outline.
  occupancyLayoutKey = "";

  // Draw the room now instead of on the next animation frame, so it shows
  // before any occupancy grid work that arrives right behind it.
  renderer.render(scene, camera);
}

// Height the TurtleBot's 2D scan is drawn at (its lidar is ~0.19 m up).
const LIDAR2_HEIGHT = 0.19;

// A device's drawn position and marker rotation. TurtleBots send Transform2
// (Vector2.y is Z, rotation is the heading from +X toward +Z); the dog sends
// Transform3 (heading is rotation.y). Null if the device has no pose.
function devicePose(device) {
  if (device.transform2?.position) {
    const p = device.transform2.position;
    return {
      x: p.x + serverOrigin.x,
      z: p.y + serverOrigin.z,
      rotation: { x: 0, y: device.transform2.rotation, z: 0 },
    };
  }
  if (device.transform3?.position) {
    const p = device.transform3.position;
    const r = device.transform3.rotation ?? { x: 0, y: 0, z: 0 };
    return { x: p.x + serverOrigin.x, z: p.z + serverOrigin.z, rotation: { x: r.x, y: r.y, z: r.z } };
  }
  return null;
}

// A device's live scan as drawn {x, y, z} points: Lidar2 for TurtleBots
// (drawn at LIDAR2_HEIGHT), Lidar3 for the dog.
function deviceScanPoints(device) {
  if (device.lidar2?.points?.length) {
    return device.lidar2.points.map((p) => ({
      x: p.x + serverOrigin.x, y: LIDAR2_HEIGHT, z: p.y + serverOrigin.z,
    }));
  }
  return (device.lidar3?.points ?? []).map((p) => ({
    x: p.x + serverOrigin.x, y: p.y, z: p.z + serverOrigin.z,
  }));
}

// Store 3D markers by device ID
const robotMarkers = {};

// Magenta stands apart from free green, occupied orange, scan coral, and map cyan.
const ROBOT_COLOR = 0xe040fb;
const ROBOT_COLOR_CSS = "#e040fb";

// Flat chevron lying in the marker's XZ plane, pointing along local +X
function createChevron(material) {
  const shape = new THREE.Shape();
  shape.moveTo(0.18, 0);      // tip
  shape.lineTo(0, 0.14);
  shape.lineTo(0.05, 0);      // inner notch
  shape.lineTo(0, -0.14);
  shape.closePath();

  const chevron = new THREE.Mesh(new THREE.ShapeGeometry(shape), material);
  // ShapeGeometry is drawn in XY; lay it flat so it reads from above.
  chevron.rotation.x = -Math.PI / 2;
  chevron.position.set(0.25, 0, 0);
  return chevron;
}

// Helper function to create or update robot marker
function updateRobotMarker(key, pose) {
  const position = pose;
  const rotation = pose.rotation;

  if (robotMarkers[key]) {
    // Update existing marker position
    robotMarkers[key].position.set(position.x, 0.3, position.z);
    // Clockwise quarter turn from the aligned heading, so the chevron faces the top wall.
    robotMarkers[key].rotation.set(rotation.x, -rotation.y, rotation.z);
  } else {
    // Create new marker
    const material = new THREE.MeshBasicMaterial({
      color: ROBOT_COLOR,
      side: THREE.DoubleSide
    });
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.2, 32, 32), material);
    marker.add(createChevron(material));
    marker.position.set(position.x, 0.3, position.z);
    marker.rotation.set(rotation.x, -rotation.y, rotation.z);
    scene.add(marker);
    robotMarkers[key] = marker;
  }
}

// LiDAR scan markers
// One instanced mesh draws all the dots without creating a separate mesh per point.
// The dots stay smaller than a grid cell so the occupancy color under them shows.
const lidarGeometry = new THREE.SphereGeometry(0.02, 8, 6);
const lidarMaterial = new THREE.MeshBasicMaterial({ color: 0xff7043, depthTest: false, transparent: true });
const lidarDot = new THREE.Object3D();
let lidarMarkers = null;
let lidarCapacity = 0;

function updateLidarMarkers(devices) {
  const points = devices.flatMap(deviceScanPoints);

  if (points.length > lidarCapacity) {
    if (lidarMarkers) {
      scene.remove(lidarMarkers);
      lidarMarkers.dispose();
    }
    lidarCapacity = points.length;
    lidarMarkers = new THREE.InstancedMesh(lidarGeometry, lidarMaterial, lidarCapacity);
    lidarMarkers.renderOrder = 2;
    scene.add(lidarMarkers);
  }

  if (lidarMarkers) points.forEach((point, index) => {
    // Lift floor-level readings slightly so they remain visible above the floor.
    lidarDot.position.set(point.x, point.y, point.z);
    lidarDot.updateMatrix();
    lidarMarkers.setMatrixAt(index, lidarDot.matrix);
  });

  // Limit the visible instances to this scan, including zero after a timeout.
  if (lidarMarkers) {
    lidarMarkers.count = points.length;
    lidarMarkers.instanceMatrix.needsUpdate = true;
    lidarMarkers.computeBoundingSphere();
  }
  updateLidarLine(devices);
}

// One polyline through each robot's scan, in the order the points were sent.
// Drawn as separate segments per device so two robots' scans are never joined.
const lidarLine = new THREE.LineSegments(
  new THREE.BufferGeometry().setAttribute("position", new THREE.BufferAttribute(new Float32Array(0), 3)),
  new THREE.LineBasicMaterial({ color: 0xff7043, depthTest: false, transparent: true })
);
lidarLine.renderOrder = 2;
lidarLine.visible = false;
scene.add(lidarLine);

function updateLidarLine(devices) {
  const coordinates = [];
  devices.forEach(device => {
    const points = deviceScanPoints(device);
    for (let index = 0; index + 1 < points.length; index += 1) {
      const start = points[index];
      const end = points[index + 1];
      coordinates.push(start.x, start.y, start.z, end.x, end.y, end.z);
    }
  });
  lidarLine.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coordinates), 3));
  lidarLine.geometry.computeBoundingSphere();
  lidarLine.visible = coordinates.length > 0;
}

// Accumulated localization map
// The server's ICP map, drawn as small dots on the floor under the live scan.
const MAP_POINT_HEIGHT = 0.03;
const mapGeometry = new THREE.SphereGeometry(0.015, 6, 4);
const mapMaterial = new THREE.MeshBasicMaterial({ color: 0x4fc3f7 });
const mapDot = new THREE.Object3D();
let mapMarkers = null;
let mapCapacity = 0;

function updateMapMarkers(points) {
  if (points.length > mapCapacity) {
    if (mapMarkers) {
      scene.remove(mapMarkers);
      mapMarkers.dispose();
    }
    // The map only grows, so leave headroom instead of reallocating every poll.
    mapCapacity = Math.max(points.length, mapCapacity * 2);
    mapMarkers = new THREE.InstancedMesh(mapGeometry, mapMaterial, mapCapacity);
    scene.add(mapMarkers);
  }

  if (!mapMarkers) return;

  // Lidar2 points in server coordinates: Vector2.y is Z.
  points.forEach((point, index) => {
    mapDot.position.set(point.x + serverOrigin.x, MAP_POINT_HEIGHT, point.y + serverOrigin.z);
    mapDot.updateMatrix();
    mapMarkers.setMatrixAt(index, mapDot.matrix);
  });
  mapMarkers.count = points.length;
  mapMarkers.instanceMatrix.needsUpdate = true;
  mapMarkers.computeBoundingSphere();
}

// Occupancy grid. The room cell lattice is built once in the browser from
// roomCorners. The server sends cells changed since occupancyRevision, the
// revision last applied here. reset=true wipes colors back to unknown (0.5).
const occupancyGeometry = new THREE.PlaneGeometry(1, 1);
const occupancyMaterial = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
const occupancyMarker = new THREE.Object3D();
const freeColor = new THREE.Color(0x2fbf7a);
const occupiedColor = new THREE.Color(0xf0a030);
const unknownColor = new THREE.Color(0x55606e);
const cellColor = new THREE.Color();
let occupancyMesh = null;
let occupancyIndexByKey = new Map();
let occupancyLayoutKey = "";
let occupancyRevision = 0n;

// Remove the occupancy lattice; the next grid fetched starts from scratch.
function clearOccupancyLattice() {
  if (occupancyMesh) {
    scene.remove(occupancyMesh);
    occupancyMesh.dispose();
    occupancyMesh = null;
  }
  occupancyIndexByKey = new Map();
  occupancyLayoutKey = "";
  occupancyRevision = 0n;
}

function pointInRoomPolygon(x, z, corners) {
  let inside = false;
  let prev = corners[corners.length - 1];
  for (const corner of corners) {
    if ((corner.z > z) !== (prev.z > z)) {
      const crossingX = ((prev.x - corner.x) * (z - corner.z)) / (prev.z - corner.z) + corner.x;
      if (x < crossingX) inside = !inside;
    }
    prev = corner;
  }
  return inside;
}

// 0.5 is flat gray. Below that, green strengthens toward 0. Above it, orange
// strengthens toward 1.
function colorForProbability(probability) {
  const p = Math.min(1, Math.max(0, probability));
  if (p < 0.5) {
    return cellColor.copy(unknownColor).lerp(freeColor, (0.5 - p) / 0.5);
  }
  if (p > 0.5) {
    return cellColor.copy(unknownColor).lerp(occupiedColor, (p - 0.5) / 0.5);
  }
  return cellColor.copy(unknownColor);
}

// Returns true when the lattice was (re)built, which leaves every cell unknown.
function ensureOccupancyLattice(cellSize, originX, originZ) {
  if (roomCorners.length < 3) return false;
  const key = `${cellSize}:${originX}:${originZ}`;
  if (occupancyMesh && occupancyLayoutKey === key) return false;

  if (occupancyMesh) {
    scene.remove(occupancyMesh);
    occupancyMesh.dispose();
  }
  occupancyLayoutKey = key;
  occupancyIndexByKey = new Map();

  // The server's room cells (map/room.py cells_covering_room) use its corners
  // stored as 32-bit floats, so round the same way to get the same cells.
  const serverCorners = roomCorners.map((c) => ({
    x: Math.fround(c.x - originX),
    z: Math.fround(c.z - originZ),
  }));
  const xs = serverCorners.map((c) => c.x);
  const zs = serverCorners.map((c) => c.z);
  const iMin = Math.floor(Math.min(...xs) / cellSize);
  const iMax = Math.floor(Math.max(...xs) / cellSize);
  const jMin = Math.floor(Math.min(...zs) / cellSize);
  const jMax = Math.floor(Math.max(...zs) / cellSize);

  const coords = [];
  for (let i = iMin; i <= iMax; i += 1) {
    for (let j = jMin; j <= jMax; j += 1) {
      const samples = [
        [(i + 0.5) * cellSize, (j + 0.5) * cellSize],
        [i * cellSize, j * cellSize],
        [(i + 1) * cellSize, j * cellSize],
        [i * cellSize, (j + 1) * cellSize],
        [(i + 1) * cellSize, (j + 1) * cellSize],
      ];
      if (samples.some(([sx, sz]) => pointInRoomPolygon(sx, sz, serverCorners))) {
        coords.push([i, j]);
      }
    }
  }

  occupancyMesh = new THREE.InstancedMesh(occupancyGeometry, occupancyMaterial, coords.length);
  occupancyMarker.rotation.set(-Math.PI / 2, 0, 0);
  occupancyMarker.scale.set(cellSize * 0.96, cellSize * 0.96, 1);

  coords.forEach(([cx, cz], index) => {
    occupancyIndexByKey.set(`${cx},${cz}`, index);
    occupancyMarker.position.set(
      (cx + 0.5) * cellSize + originX,
      0.015,
      (cz + 0.5) * cellSize + originZ
    );
    occupancyMarker.updateMatrix();
    occupancyMesh.setMatrixAt(index, occupancyMarker.matrix);
    occupancyMesh.setColorAt(index, unknownColor);
  });

  occupancyMesh.count = coords.length;
  occupancyMesh.instanceMatrix.needsUpdate = true;
  if (occupancyMesh.instanceColor) occupancyMesh.instanceColor.needsUpdate = true;
  if (coords.length) occupancyMesh.computeBoundingSphere();
  scene.add(occupancyMesh);
  return true;
}

// Apply an OccupancyGrid message. Returns false when it could not be applied
// (no room yet, or a fresh lattice that a delta cannot fill in).
function updateOccupancyGrid(grid) {
  if (!(grid.cellSize > 0)) return false;
  // cellSize arrives as a 32-bit float; the server's is the decimal value (0.1).
  const cellSize = Number(grid.cellSize.toPrecision(6));
  // The grid is in server coordinates, drawn shifted by the room's origin.
  const rebuilt = ensureOccupancyLattice(cellSize, serverOrigin.x, serverOrigin.z);
  if (!occupancyMesh) return false;
  if (rebuilt && !grid.reset) return false;

  // Full wipe after OccupancyGrid.clear on the server, or a fresh start.
  if (grid.reset) {
    for (let index = 0; index < occupancyMesh.count; index += 1) {
      occupancyMesh.setColorAt(index, unknownColor);
    }
  }

  // Apply this file's delta on top of colors already on the lattice.
  for (const cell of grid.cells) {
    const index = occupancyIndexByKey.get(`${cell.x},${cell.z}`);
    if (index === undefined) continue;
    if (!Number.isFinite(cell.p)) continue;
    occupancyMesh.setColorAt(index, colorForProbability(cell.p));
  }

  if (occupancyMesh.instanceColor) occupancyMesh.instanceColor.needsUpdate = true;
  return true;
}

// Rendering and window behavior
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

function resizeRenderer() {
  camera.aspect = sceneContainer.clientWidth / sceneContainer.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(sceneContainer.clientWidth, sceneContainer.clientHeight);
}

window.addEventListener("resize", resizeRenderer);

// Server connection
// The dashboard is served from the same machine as the realityapi server.
const SERVER_URL = `ws://${location.hostname || "localhost"}:65432`;

// How long to wait between FetchAllDeviceInfo polls (ms).
const POLL_INTERVAL = 50;

// How long to wait between FetchLidarData polls for the accumulated map (ms).
// The map changes slowly and is much larger than the device list.
const MAP_POLL_INTERVAL = 500;

// How long to wait between occupancy grid polls (ms).
const OCCUPANCY_POLL_INTERVAL = 1000;

// How long to wait before reconnecting after the socket closes (ms).
const RECONNECT_DELAY = 1000;

let socket = null;
let nextRequestId = 0;
// Callbacks for requests still waiting on a Response, keyed by request id.
const pendingRequests = new Map();

function connect() {
  socket = new WebSocket(SERVER_URL);
  socket.binaryType = "arraybuffer";

  socket.addEventListener("message", (event) => {
    if (typeof event.data === "string") {
      const response = JSON.parse(event.data);
      const pending = pendingRequests.get(response.id);
      if (pending) {
        pendingRequests.delete(response.id);
        pending.resolve(response);
      }
      return;
    }
    const response = fromBinary(ResponseSchema, new Uint8Array(event.data));
    const pending = pendingRequests.get(response.reqUuid);
    if (pending) {
      pendingRequests.delete(response.reqUuid);
      pending.resolve(response);
    }
  });

  socket.addEventListener("open", () => {
    fetchRoomBounds();
    loadTunables();
  });

  socket.addEventListener("close", () => {
    for (const pending of pendingRequests.values()) {
      pending.reject(new Error("connection closed"));
    }
    pendingRequests.clear();
    setTimeout(connect, RECONNECT_DELAY);
  });
}

// Send one ServerRequest and resolve with the matching Response.
function request(data) {
  const reqUuid = String(nextRequestId++);
  const message = create(ServerRequestSchema, { reqUuid, data });

  return new Promise((resolve, reject) => {
    pendingRequests.set(reqUuid, { resolve, reject });
    socket.send(toBinary(ServerRequestSchema, message));
  });
}

// Ask the server for the room outline and obstacles and redraw them.
async function fetchRoomBounds() {
  try {
    const response = await request({ case: "fetchRoomBounds", value: {} });
    if (response.data.case === "roomBounds") {
      buildRoom(response.data.value);
    }
  } catch (error) {
    console.error("Error fetching room bounds:", error);
  }
}

// Show another device type's occupancy grid.
function setOccupancyDeviceType(deviceType) {
  occupancyDeviceType = deviceType;
  const url = new URL(location.href);
  url.searchParams.set("device", DeviceType[deviceType]);
  history.replaceState(null, "", url);
  // The next poll fetches the new type's grid from scratch.
  clearOccupancyLattice();
}

const occupancyDevicePicker = document.getElementById("occupancy-device-type");
occupancyDevicePicker?.addEventListener("change", () => setOccupancyDeviceType(Number(occupancyDevicePicker.value)));
let occupancyPickerKey = "";

// List the device types that are connected now, plus the one being shown.
function updateOccupancyPicker(devices) {
  if (!occupancyDevicePicker) return;
  const types = new Set(devices.map((device) => device.deviceType).filter((type) => type !== DeviceType.NONE));
  types.add(occupancyDeviceType);
  const sorted = [...types].sort((a, b) => a - b);
  const key = sorted.join(",");
  if (key === occupancyPickerKey) return;
  occupancyPickerKey = key;
  occupancyDevicePicker.replaceChildren(
    ...sorted.map((type) => new Option(DeviceType[type], String(type), false, type === occupancyDeviceType))
  );
}
updateOccupancyPicker([]);

// Track device positions
let lastPositions = {};

function updateDevices(devices) {
  const currentKeys = new Set();
  devices.forEach(device => {
    // Only devices that report a pose have a place on the map.
    const position = devicePose(device);
    if (!position) return;

    const key = `${device.id}-${device.deviceType}`;
    currentKeys.add(key);

    const newPos = { x: position.x, z: position.z };

    // Update 3D marker
    updateRobotMarker(key, position);

    // Check if position changed or device is new
    if (!lastPositions[key] ||
      (Math.abs(lastPositions[key].x - newPos.x) > 0.01 || Math.abs(lastPositions[key].z - newPos.z) > 0.01)) {
      // Update log
      const logContainer = document.getElementById("log-container");
      const logEntry = document.createElement("div");
      logEntry.className = "log-entry";
      const status = lastPositions[key] ? "moved" : "connected";
      logEntry.innerHTML = `
        <span class="name" style="color: ${ROBOT_COLOR_CSS}">${device.id}</span>
        <span class="status" style="color: ${ROBOT_COLOR_CSS}">${DeviceType[device.deviceType]} ${status}</span>
        <div class="coordinates">X: ${position.x.toFixed(2)} m · Z: ${position.z.toFixed(2)} m</div>
      `;
      logContainer.appendChild(logEntry);
      // Auto-scroll to bottom (scroll the dashboard parent)
      const dashboard = document.querySelector('.dashboard');
      dashboard.scrollTop = dashboard.scrollHeight;
    }

    lastPositions[key] = newPos;
  });

  // Remove devices the server no longer reports
  Object.keys(robotMarkers).forEach(key => {
    if (!currentKeys.has(key)) {
      scene.remove(robotMarkers[key]);
      delete robotMarkers[key];
      delete lastPositions[key];
    }
  });
}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

// Ask the server for every device's state, one request at a time.
async function pollDevices() {
  while (true) {
    if (socket.readyState === WebSocket.OPEN) {
      try {
        const response = await request({ case: "fetchAllDeviceInfo", value: {} });
        if (response.data.case === "allDeviceInfo") {
          updateDevices(response.data.value.devices);
          updateLidarMarkers(response.data.value.devices);
          updateOccupancyPicker(response.data.value.devices);
        }
      } catch (error) {
        console.error("Error fetching devices:", error);
      }
    }
    await sleep(POLL_INTERVAL);
  }
}

// Ask the server for the accumulated localization map.
async function pollMap() {
  while (true) {
    if (socket.readyState === WebSocket.OPEN) {
      try {
        const response = await request({ case: "fetchLidarData", value: {} });
        if (response.data.case === "lidarData") {
          updateMapMarkers(response.data.value.points);
        }
      } catch (error) {
        console.error("Error fetching map:", error);
      }
    }
    await sleep(MAP_POLL_INTERVAL);
  }
}

// Ask the server for occupancy cells changed since the revision applied here.
async function fetchOccupancyGrid() {
  const status = document.getElementById("occupancy-status");
  // The lattice is built from the room outline, so there is nothing to apply
  // cells to yet. The room itself never waits for the grid.
  if (roomCorners.length < 3) {
    if (status) status.textContent = "Waiting for room layout";
    return;
  }
  const deviceType = occupancyDeviceType;
  const response = await request({
    case: "fetchOccupancyGrid",
    value: { deviceType, sinceRevision: occupancyRevision },
  });
  if (deviceType !== occupancyDeviceType) return;  // the picker moved on meanwhile
  if (response.data.case !== "occupancyGrid") {
    clearOccupancyLattice();
    if (status) status.textContent = `No occupancy grid for ${DeviceType[deviceType]}`;
    return;
  }
  const grid = response.data.value;
  if (updateOccupancyGrid(grid)) {
    occupancyRevision = grid.revision;
    if (status) status.textContent = "Occupancy grid loaded";
  } else {
    // Nothing was applied, so ask for every cell next time.
    occupancyRevision = 0n;
    if (status) status.textContent = "Waiting for occupancy grid";
  }
}

async function pollOccupancyGrid() {
  while (true) {
    if (socket.readyState === WebSocket.OPEN) {
      try {
        await fetchOccupancyGrid();
      } catch (error) {
        const status = document.getElementById("occupancy-status");
        if (status) status.textContent = "Occupancy grid unavailable";
        console.error("Error fetching occupancy grid:", error);
      }
    }
    await sleep(OCCUPANCY_POLL_INTERVAL);
  }
}

// Fetch the server's accumulated map and save it as a JSON file of {x, y, z} points.
async function downloadPointCloud() {
  const button = document.getElementById("download-scan");
  if (!button) return;
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    console.warn("Download point cloud: not connected to the server");
    return;
  }

  button.disabled = true;
  try {
    const response = await request({ case: "fetchLidarData", value: {} });
    if (response.data.case !== "lidarData") {
      console.warn("Download point cloud: unexpected reply from server:", response.data.case);
      return;
    }

    // Copy plain drawn coordinates so protobuf message metadata stays out of
    // the file. Lidar2 points are server X/Z (Vector2.y is Z) on the floor.
    const points = response.data.value.points.map(({ x, y }) => ({
      x: x + serverOrigin.x, y: 0, z: y + serverOrigin.z,
    }));
    const blob = new Blob([JSON.stringify(points)], { type: "application/json" });
    const blobUrl = URL.createObjectURL(blob);

    const link = document.createElement("a");
    link.href = blobUrl;
    link.download = "pointCloudData.json";
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(blobUrl);
  } catch (error) {
    console.error("Error downloading point cloud:", error);
  } finally {
    button.disabled = false;
  }
}

async function clearAccumulatedScan() {
  const button = document.getElementById("clear-scan");
  if (!button || !socket || socket.readyState !== WebSocket.OPEN) return;

  button.disabled = true;
  try {
    const response = await request({ case: "clearAccumulatedScan", value: {} });
    if (response.data.case === "ok") {
      updateMapMarkers([]);
      try {
        await fetchOccupancyGrid();
      } catch (error) {
        console.error("Error reloading occupancy grid:", error);
      }
    } else {
      console.warn("Clear accumulated scan was not accepted; server replied:", response.data.case);
    }
  } catch (error) {
    console.error("Error clearing accumulated scan:", error);
  } finally {
    button.disabled = false;
  }
}

document.getElementById("clear-scan")?.addEventListener("click", () => {
  clearAccumulatedScan();
});

document.getElementById("download-scan")?.addEventListener("click", () => {
  downloadPointCloud();
});

document.getElementById("save-rmse-histogram")?.addEventListener("click", () => {
  saveRmseHistogram();
});

async function saveRmseHistogram() {
  const button = document.getElementById("save-rmse-histogram");
  const status = document.getElementById("rmse-status");
  if (!button || !status || !socket || socket.readyState !== WebSocket.OPEN) {
    if (status) status.textContent = "Server disconnected. Reconnect and try again.";
    return;
  }
  button.disabled = true;
  status.textContent = "Building histogram…";
  try {
    const response = await dashboardRequest({ action: "save_rmse_histogram" });
    if (!response.ok) throw new Error(response.error || "Server could not save the histogram");
    status.textContent = `Saved histogram of ${response.count} scans to ${response.path}`;
  } catch (error) {
    status.textContent = `Could not complete action: ${error.message}`;
  } finally {
    button.disabled = false;
  }
}

function dashboardRequest(fields) {
  const id = `dash-${nextRequestId++}`;
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      pendingRequests.delete(id);
      reject(new Error("Server did not respond; restart it and try again"));
    }, 10000);
    pendingRequests.set(id, {
      resolve: value => { clearTimeout(timeout); resolve(value); },
      reject: error => { clearTimeout(timeout); reject(error); },
    });
    socket.send(JSON.stringify({ id, ...fields }));
  });
}

function decimalsFor(step) {
  const text = String(step);
  const dot = text.indexOf(".");
  return dot === -1 ? 0 : text.length - dot - 1;
}

async function loadTunables() {
  const root = document.getElementById("tunables");
  if (!root || !socket || socket.readyState !== WebSocket.OPEN) return;
  try {
    const response = await dashboardRequest({ action: "fetch_tunables" });
    if (!response.ok) throw new Error(response.error || "Could not load parameters");
    renderTunables(response.parameters || []);
  } catch (error) {
    root.textContent = error.message;
  }
}

function renderTunables(parameters) {
  const root = document.getElementById("tunables");
  root.replaceChildren();
  for (const parameter of parameters) {
    const digits = decimalsFor(parameter.step);
    const row = document.createElement("label");
    row.className = "tunable";
    const title = document.createElement("span");
    title.textContent = parameter.label;
    const value = document.createElement("span");
    value.className = "tunable-value";
    const suffix = parameter.unit ? ` ${parameter.unit}` : "";
    const show = number => {
      value.textContent = `${Number(number).toFixed(digits)}${suffix}`;
    };
    show(parameter.value);
    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = String(parameter.minimum);
    slider.max = String(parameter.maximum);
    slider.step = String(parameter.step);
    slider.value = String(parameter.value);
    slider.setAttribute("aria-label", parameter.label);
    slider.addEventListener("input", () => show(slider.value));
    slider.addEventListener("change", async () => {
      try {
        const response = await dashboardRequest({
          action: "set_tunable",
          name: parameter.name,
          value: Number(slider.value),
        });
        if (!response.ok) throw new Error(response.error || "Could not update parameter");
        slider.value = String(response.parameter.value);
        show(response.parameter.value);
      } catch (error) {
        value.textContent = error.message;
      }
    });
    row.append(title, value, slider);
    root.append(row);
  }
}

connect();
pollDevices();
pollMap();
pollOccupancyGrid();
animate();