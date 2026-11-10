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

// Room floor
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

// Store 3D markers by device ID
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

// LiDAR scan markers
// One instanced mesh draws all the dots without creating a separate mesh per point.
// Bigger dots, drawn on top of the room and the accumulated map, so the live scan
// is not lost against them from the default camera distance.
const lidarGeometry = new THREE.SphereGeometry(0.1, 8, 6);
const lidarMaterial = new THREE.MeshBasicMaterial({ color: 0xff7043, depthTest: false, transparent: true });
const lidarDot = new THREE.Object3D();
let lidarMarkers = null;
let lidarCapacity = 0;

function updateLidarMarkers(devices) {
  const points = devices.flatMap(device => device.lidar?.points ?? []);

  console.log("LiDAR point count:", points.length, "First point:", points[0]);

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
    const points = device.lidar?.points ?? [];
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
const mapGeometry = new THREE.SphereGeometry(0.05, 6, 4);
const mapMaterial = new THREE.MeshBasicMaterial({ color: 0x4fc3f7 });
const mapDot = new THREE.Object3D();
let mapMarkers = null;
let mapCapacity = 0;

let lastMapCount = -1;

function updateMapMarkers(points) {
  if (points.length !== lastMapCount) {
    lastMapCount = points.length;
    console.log("Accumulated map points:", points.length);
  }
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

  points.forEach((point, index) => {
    mapDot.position.set(point.x, MAP_POINT_HEIGHT, point.z);
    mapDot.updateMatrix();
    mapMarkers.setMatrixAt(index, mapDot.matrix);
  });
  mapMarkers.count = points.length;
  mapMarkers.instanceMatrix.needsUpdate = true;
  mapMarkers.computeBoundingSphere();
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

// Track device positions
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

async function clearAccumulatedScan() {
  const button = document.getElementById("clear-scan");
  if (!button || !socket || socket.readyState !== WebSocket.OPEN) return;

  button.disabled = true;
  console.log("Clear accumulated scan: button clicked, sending request");
  try {
    const response = await request({ case: "clearAccumulatedScan", value: {} });
    if (response.data.case === "ok") {
      console.log("Clear accumulated scan: server accepted");
      updateMapMarkers([]);
    } else {
      console.warn("Clear accumulated scan was not accepted; server replied:", response.data.case);
    }
  } catch (error) {
    console.error("Error clearing accumulated scan:", error);
  } finally {
    button.disabled = false;
  }
}

if (!document.getElementById("clear-scan")) {
  const clearButton = document.createElement("button");
  clearButton.id = "clear-scan";
  clearButton.type = "button";
  clearButton.textContent = "Clear accumulated scan";
  Object.assign(clearButton.style, {
    position: "absolute",
    top: "12px",
    right: "12px",
    zIndex: "10",
    padding: "12px 16px",
    font: "16px system-ui, sans-serif",
    color: "#fff",
    background: "#0f1115",
    border: "none",
    borderRadius: "10px",
    cursor: "pointer"
  });
  if (getComputedStyle(sceneContainer).position === "static") {
    sceneContainer.style.position = "relative";
  }
  sceneContainer.appendChild(clearButton);
}

document.getElementById("clear-scan")?.addEventListener("click", () => {
  clearAccumulatedScan();
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
  const id = `rmse-${nextRequestId++}`;
  try {
    const response = await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        pendingRequests.delete(id);
        reject(new Error("Server did not respond; restart it and try again"));
      }, 10000);
      pendingRequests.set(id, {
        resolve: value => { clearTimeout(timeout); resolve(value); },
        reject: error => { clearTimeout(timeout); reject(error); },
      });
      socket.send(JSON.stringify({ id, action: "save_rmse_histogram" }));
    });
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
    const show = number => {
      value.textContent = `${Number(number).toFixed(digits)} m`;
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
animate();