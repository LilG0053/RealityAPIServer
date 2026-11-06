import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { create, fromBinary, toBinary } from "@bufbuild/protobuf";
import { DeviceType, ResponseSchema, ServerRequestSchema } from "./gen/realityapi_pb.js";

// Dashboard elements and room dimensions
const sceneContainer = document.getElementById("scene-container");

// One Three.js unit represents one real-world meter.
// Width is X, height is Y, depth is Z.
const WALL_HEIGHT = 3;
const WALL_THICKNESS = 0.15;

// Room corners (in meters)
// (0, 0) is the corner furthest from the door.
const roomCorners = [
  { x: 0, z: 0 },
  { x: 0, z: 15.45 },
  { x: 5.16, z: 15.45 },
  { x: 5.16, z: 12.26 },
  { x: 9.07, z: 12.26 },
  { x: 9.07, z: 0 }
];

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

// Camera interaction
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

// Room geometry: floor
// ShapeGeometry fills the exact L-shaped outline from roomCorners.
const floorShape = new THREE.Shape();
floorShape.moveTo(roomCorners[0].x, roomCorners[0].z);

for (const corner of roomCorners.slice(1)) {
  floorShape.lineTo(corner.x, corner.z);
}

floorShape.closePath();

const floor = new THREE.Mesh(
  new THREE.ShapeGeometry(floorShape),
  new THREE.MeshBasicMaterial({
    color: 0x68717d,
    side: THREE.DoubleSide
  })
);
floor.rotation.x = Math.PI / 2;
scene.add(floor);

// Coordinate axes and labels
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

// Room geometry: walls and outline
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
  scene.add(wall);
}

for (let index = 0; index < roomCorners.length; index += 1) {
  const start = roomCorners[index];
  const end = roomCorners[(index + 1) % roomCorners.length];
  createWallBetween(start, end);
}

const roomOutline = new THREE.LineLoop(
  new THREE.BufferGeometry().setFromPoints(
    roomCorners.map((corner) => new THREE.Vector3(corner.x, 0.02, corner.z))
  ),
  new THREE.LineBasicMaterial({ color: 0xffffff })
);
scene.add(roomOutline);

// Robot markers and device colors
const robotMarkers = {};

// Generate consistent color from string
function stringToColor(str) {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    hash = str.charCodeAt(i) + ((hash << 5) - hash);
  }
  const c = (hash & 0x00FFFFFF).toString(16).toUpperCase();
  return "#" + "00000".substring(0, 6 - c.length) + c;
}

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
function updateRobotMarker(key, device) {
  const position = device.transform.position;
  const rotation = device.transform.rotation
  const color = stringToColor(device.id);

  if (robotMarkers[key]) {
    // Update existing marker position
    robotMarkers[key].position.set(position.x, 0.3, position.z);
    // Clockwise quarter turn from the aligned heading, so the chevron faces the top wall.
    robotMarkers[key].rotation.set(rotation.x, -rotation.y, rotation.z);
  } else {
    // Create new marker
    const material = new THREE.MeshBasicMaterial({
      color: parseInt(color.replace('#', '0x')),
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

// Shared point-layer rendering
// Instancing keeps each layer to one draw call as point counts grow.
function createPointLayer(color, radius, minHeight) {
  const geometry = new THREE.SphereGeometry(radius, 8, 6);
  const material = new THREE.MeshBasicMaterial({ color });
  const marker = new THREE.Object3D();
  let mesh = null;
  let capacity = 0;

  return function updatePoints(points) {
    if (points.length > capacity) {
      if (mesh) {
        scene.remove(mesh);
        mesh.dispose();
      }
      capacity = points.length;
      mesh = new THREE.InstancedMesh(geometry, material, capacity);
      scene.add(mesh);
    }
    if (!mesh) return;

    points.forEach((point, index) => {
      marker.position.set(point.x, Math.max(point.y, minHeight), point.z);
      marker.updateMatrix();
      mesh.setMatrixAt(index, marker.matrix);
    });
    mesh.count = points.length;
    mesh.instanceMatrix.needsUpdate = true;
    if (points.length) mesh.computeBoundingSphere();
  };
}

// Live LiDAR scan visualization
const updateLiveScan = createPointLayer(0xff4d4d, 0.045, 0.10);

// Accumulated global point-map visualization
const updateGlobalPoints = createPointLayer(0x438cff, 0.035, 0.06);

// Occupancy-grid visualization
// The room floor represents unknown cells; only observed cells need meshes.
const occupancyGeometry = new THREE.PlaneGeometry(1, 1);
const occupancyMaterial = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
const occupancyMarker = new THREE.Object3D();
const occupancyColors = {
  free: new THREE.Color(0x4cbd83),
  occupied: new THREE.Color(0xe5a134),
  uncertain: new THREE.Color(0xd4cc79),
};
let occupancyMesh = null;
let occupancyCapacity = 0;

function updateOccupancyGrid(grid) {
  if (!Number.isFinite(grid.cell_size) || grid.cell_size <= 0 || !Array.isArray(grid.cells)) return;
  const cells = grid.cells;
  if (cells.length > occupancyCapacity) {
    if (occupancyMesh) {
      scene.remove(occupancyMesh);
      occupancyMesh.dispose();
    }
    occupancyCapacity = cells.length;
    occupancyMesh = new THREE.InstancedMesh(occupancyGeometry, occupancyMaterial, occupancyCapacity);
    scene.add(occupancyMesh);
  }
  if (!occupancyMesh) return;

  cells.forEach((cell, index) => {
    occupancyMarker.position.set((cell.x + 0.5) * grid.cell_size, 0.015, (cell.z + 0.5) * grid.cell_size);
    occupancyMarker.rotation.set(-Math.PI / 2, 0, 0);
    occupancyMarker.scale.set(grid.cell_size * 0.96, grid.cell_size * 0.96, 1);
    occupancyMarker.updateMatrix();
    occupancyMesh.setMatrixAt(index, occupancyMarker.matrix);
    let color;
    if (cell.status === 1) {
      color = occupancyColors.occupied;
    } else if (cell.status === 0) {
      color = occupancyColors.free;
    }
    occupancyMesh.setColorAt(index, color);
  });
  occupancyMesh.count = cells.length;
  occupancyMesh.instanceMatrix.needsUpdate = true;
  if (occupancyMesh.instanceColor) occupancyMesh.instanceColor.needsUpdate = true;
  if (cells.length) occupancyMesh.computeBoundingSphere();
}

// Global point-map data loading
// The demo uses a separate file so it cannot replace the robot's map.
const globalMapFile = new URLSearchParams(location.search).has("demo")
  ? "./global_map_demo.json"
  : "./global_map.json";

async function pollGlobalMap() {
  while (true) {
    try {
      const response = await fetch(globalMapFile, {
        cache: "no-store",
      });

      if (response.ok) {
        const points = await response.json();
        updateGlobalPoints(points);
      } else if (response.status !== 404) {
        console.error("Could not load global map:", response.status);
      }
    } catch (error) {
      console.error("Error loading global map:", error);
    }

    // The map doesn't need to refresh as often as the robot position.
    await sleep(1000);
  }
}

// Occupancy-grid data loading
async function pollOccupancyGrid() {
  while (true) {
    try {
      const response = await fetch("./occupancy_grid.json", { cache: "no-store" });
      if (response.ok) {
        const grid = await response.json();
        updateOccupancyGrid(grid);
        document.getElementById("occupancy-status").textContent = "Occupancy grid loaded";
      } else {
        document.getElementById("occupancy-status").textContent = "Waiting for occupancy grid";
        if (response.status !== 404) console.error("Could not load occupancy grid:", response.status);
      }
    } catch (error) {
      document.getElementById("occupancy-status").textContent = "Occupancy grid unavailable";
      console.error("Error loading occupancy grid:", error);
    }
    await sleep(1000);
  }
}

// Rendering and responsive window behavior
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

// WebSocket connection and requests
// The dashboard is served from the same machine as the realityapi server.
const SERVER_URL = `ws://${location.hostname || "localhost"}:65432`;

// How long to wait between FetchAllDeviceInfo polls (ms).
const POLL_INTERVAL = 50;

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
    const response = fromBinary(ResponseSchema, new Uint8Array(event.data));
    const pending = pendingRequests.get(response.reqUuid);
    if (pending) {
      pendingRequests.delete(response.reqUuid);
      pending.resolve(response);
    }
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

// Device markers and activity log
let lastPositions = {};

function updateDevices(devices) {
  const currentKeys = new Set();
  devices.forEach(device => {
    // Only devices that report a transform have a place on the map.
    if (!device.transform?.position) return;

    const key = `${device.id}-${device.deviceType}`;
    currentKeys.add(key);

    const position = device.transform.position;
    const newPos = { x: position.x, z: position.z };

    const rotation = device.transform.rotation;
    const newrot = { x: rotation.x, y: rotation.y, z: rotation.z };

    // Update 3D marker
    updateRobotMarker(key, device);

    // Check if position changed or device is new
    if (!lastPositions[key] ||
      (Math.abs(lastPositions[key].x - newPos.x) > 0.01 || Math.abs(lastPositions[key].z - newPos.z) > 0.01)) {
      // Update log
      const logContainer = document.getElementById("log-container");
      const logEntry = document.createElement("div");
      logEntry.className = "log-entry";
      const status = lastPositions[key] ? "moved" : "connected";
      const color = stringToColor(device.id);
      logEntry.innerHTML = `
        <span class="name" style="color: ${color}">${device.id}</span>
        <span class="status" style="color: ${color}">${DeviceType[device.deviceType]} ${status}</span>
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

// Device polling
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

// Ask the server for every device's state, one request at a time.
async function pollDevices() {
  while (true) {
    if (socket.readyState === WebSocket.OPEN) {
      try {
        const response = await request({ case: "fetchAllDeviceInfo", value: {} });
        if (response.data.case === "allDeviceInfo") {
          updateDevices(response.data.value.devices);
          updateLiveScan(response.data.value.devices.flatMap(device => device.lidar?.points ?? []));
        }
      } catch (error) {
        console.error("Error fetching devices:", error);
      }
    }
    await sleep(POLL_INTERVAL);
  }
}

// Start dashboard updates
connect();
pollDevices();
animate();
pollGlobalMap();
pollOccupancyGrid();
