import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { create, fromBinary, toBinary } from "@bufbuild/protobuf";
import { DeviceType, ResponseSchema, ServerRequestSchema } from "./gen/realityapi_pb.js";

console.log("Reality dashboard main.js loaded");

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
const lidarPointClouds = {};
// Upper bound on points drawn per device; keep >= REALITYAPI_LIDAR_MAX_POINTS on the client.
const MAX_LIDAR_POINTS = 40000;

// Generate consistent color from string
function stringToColor(str) {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    hash = str.charCodeAt(i) + ((hash << 5) - hash);
  }
  const c = (hash & 0x00FFFFFF).toString(16).toUpperCase();
  return "#" + "00000".substring(0, 6 - c.length) + c;
}

// Helper function to create or update robot marker
function updateRobotMarker(key, device) {
  const position = device.transform.position;
  const color = stringToColor(device.id);

  if (robotMarkers[key]) {
    // Update existing marker position
    robotMarkers[key].position.set(position.x, 0.3, position.z);
  } else {
    // Create new marker
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.2, 32, 32),
      new THREE.MeshBasicMaterial({ color: parseInt(color.replace('#', '0x')) })
    );
    marker.position.set(position.x, 0.3, position.z);
    scene.add(marker);
    robotMarkers[key] = marker;
  }
}

function updateLidarPointCloud(key, device) {
  const points = device.lidar?.points;
  const position = device.transform?.position;
  if (!points?.length || !position) {
    const previous = lidarPointClouds[key];
    if (previous) {
      scene.remove(previous);
      previous.geometry.dispose();
      previous.material.dispose();
      delete lidarPointClouds[key];
    }
    return;
  }

  const count = Math.min(points.length, MAX_LIDAR_POINTS);

  let pointCloud = lidarPointClouds[key];
  if (!pointCloud) {
    // Allocate the GPU buffer once at full size and reuse it for every update.
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(new Float32Array(MAX_LIDAR_POINTS * 3), 3)
    );
    pointCloud = new THREE.Points(
      geometry,
      new THREE.PointsMaterial({
        color: stringToColor(device.id),
        size: 0.04,
        sizeAttenuation: true
      })
    );
    // The preallocated buffer's bounding sphere would be wrong, so skip culling.
    pointCloud.frustumCulled = false;
    scene.add(pointCloud);
    lidarPointClouds[key] = pointCloud;
  }

  const attribute = pointCloud.geometry.getAttribute("position");
  const coordinates = attribute.array;
  const px = position.x;
  const py = position.y;
  const pz = position.z;
  for (let index = 0, offset = 0; index < count; index += 1, offset += 3) {
    const point = points[index];
    coordinates[offset] = px + point.x;
    coordinates[offset + 1] = py + point.y;
    coordinates[offset + 2] = pz + point.z;
  }
  attribute.needsUpdate = true;
  pointCloud.geometry.setDrawRange(0, count);
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
const POLL_INTERVAL = 10;

// How long to wait before reconnecting after the socket closes (ms).
const RECONNECT_DELAY = 1000;

let socket = null;
let nextRequestId = 0;
// Callbacks for requests still waiting on a Response, keyed by request id.
const pendingRequests = new Map();

function connect() {
  console.log("Opening dashboard WebSocket:", SERVER_URL);
  socket = new WebSocket(SERVER_URL);
  socket.binaryType = "arraybuffer";

  socket.addEventListener("open", () => {
    console.log("Dashboard WebSocket opened:", SERVER_URL);
  });

  socket.addEventListener("message", (event) => {
    // No per-message console logging here: responses carry thousands of lidar
    // points and logging them (devtools keeps and serializes the objects) is
    // very slow.
    let response;
    try {
      response = fromBinary(ResponseSchema, new Uint8Array(event.data));
    } catch (error) {
      console.error("Failed to decode dashboard response:", error);
      return;
    }

    const pending = pendingRequests.get(response.reqUuid);
    if (pending) {
      pendingRequests.delete(response.reqUuid);
      pending.resolve(response);
    } else {
      console.warn("Received response with no pending request:", response.reqUuid);
    }
  });

  socket.addEventListener("error", (event) => {
    console.error("Dashboard WebSocket error:", event);
  });

  socket.addEventListener("close", () => {
    console.warn("Dashboard WebSocket closed; reconnecting:", SERVER_URL);
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

    // Update 3D marker
    updateRobotMarker(key, device);
    updateLidarPointCloud(key, device);

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
      if (lidarPointClouds[key]) {
        scene.remove(lidarPointClouds[key]);
        lidarPointClouds[key].geometry.dispose();
        lidarPointClouds[key].material.dispose();
        delete lidarPointClouds[key];
      }
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
        } else {
          console.warn("Unexpected dashboard response case:", response.data.case);
        }
      } catch (error) {
        console.error("Error fetching devices:", error);
      }
    }
    await sleep(POLL_INTERVAL);
  }
}
connect();
pollDevices();
animate();