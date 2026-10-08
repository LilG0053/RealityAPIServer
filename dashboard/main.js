import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// === Dashboard DOM and room measurements ===

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

// === Three.js scene setup ===

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

// === Camera controls and lighting ===

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

// === Room geometry ===

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

// === Robot markers on the room map ===

// Store 3D markers by device ID.
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

// Helper function to create or update robot marker
function updateRobotMarker(device) {
  const key = `${device.id}-${device.device_type}`;
  const color = stringToColor(device.id);

  if (robotMarkers[key]) {
    // Update existing marker position
    robotMarkers[key].position.set(device.pos.x, 0.3, device.pos.z);
  } else {
    // Create new marker
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.2, 32, 32),
      new THREE.MeshBasicMaterial({ color: parseInt(color.replace('#', '0x')) })
    );
    marker.position.set(device.pos.x, 0.3, device.pos.z);
    scene.add(marker);
    robotMarkers[key] = marker;
  }
}

// === Rendering and window behavior ===

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

// === Device activity cards and browser-saved history ===

// A device keeps one dashboard card with movement added beneath it.
const deviceActivityPanels = {};
const ACTIVITY_HISTORY_STORAGE_KEY = "robot-room-device-activity";
const MAX_ACTIVITY_EVENTS_PER_DEVICE = 40;
let nextActivityListId = 0;

function loadDeviceActivityHistory() {
  try {
    const savedHistory = sessionStorage.getItem(ACTIVITY_HISTORY_STORAGE_KEY);
    const parsedHistory = savedHistory ? JSON.parse(savedHistory) : {};

    return parsedHistory && typeof parsedHistory === "object" && !Array.isArray(parsedHistory)
      ? parsedHistory
      : {};
  } catch (error) {
    console.warn("Could not restore robot activity history:", error);
    return {};
  }
}

function saveDeviceActivityHistory() {
  sessionStorage.setItem(ACTIVITY_HISTORY_STORAGE_KEY, JSON.stringify(deviceActivityHistory));
}

let deviceActivityHistory = loadDeviceActivityHistory();

function formatCoordinates(position) {
  return `X: ${position.x.toFixed(2)} m · Z: ${position.z.toFixed(2)} m`;
}

function createDeviceActivityPanel(device, key) {
  const logContainer = document.getElementById("log-container");
  const panel = document.createElement("section");
  const header = document.createElement("header");
  const name = document.createElement("h2");
  const headerActions = document.createElement("div");
  const status = document.createElement("span");
  const toggle = document.createElement("button");
  const toggleIcon = document.createElement("span");
  const latestPosition = document.createElement("p");
  const activityList = document.createElement("ul");
  const color = stringToColor(device.id);

  panel.className = "device-activity";
  panel.dataset.deviceKey = key;
  header.className = "device-activity__header";
  name.className = "device-activity__name";
  name.textContent = device.id;
  name.style.setProperty("--device-color", color);
  headerActions.className = "device-activity__actions";
  status.className = "device-activity__status";
  toggle.className = "device-activity__toggle";
  toggle.type = "button";
  toggle.setAttribute("aria-expanded", "false");
  toggle.setAttribute("aria-label", `Show movement history for ${device.id}`);
  toggleIcon.className = "device-activity__toggle-icon";
  toggleIcon.setAttribute("aria-hidden", "true");
  toggleIcon.textContent = "⌄";
  latestPosition.className = "device-activity__position";
  activityList.className = "device-activity__events";
  activityList.id = `device-activity-events-${nextActivityListId}`;
  nextActivityListId += 1;
  activityList.setAttribute("aria-live", "polite");
  activityList.setAttribute("aria-hidden", "true");
  toggle.setAttribute("aria-controls", activityList.id);

  toggle.appendChild(toggleIcon);
  headerActions.append(status, toggle);
  header.append(name, headerActions);
  panel.append(header, latestPosition, activityList);
  logContainer.appendChild(panel);

  for (const event of deviceActivityHistory[key] || []) {
    renderDeviceActivity(activityList, event);
  }

  toggle.addEventListener("click", () => {
    const isOpen = panel.classList.contains("is-history-open");
    setDeviceActivityHistoryOpen({ panel, toggle, activityList }, !isOpen, device.id);
  });

  return { panel, status, toggle, latestPosition, activityList };
}

function getDeviceActivityPanel(device, key) {
  if (deviceActivityPanels[key]) {
    return { activityPanel: deviceActivityPanels[key], isNew: false };
  }

  const activityPanel = createDeviceActivityPanel(device, key);
  deviceActivityPanels[key] = activityPanel;
  return { activityPanel, isNew: true };
}

function setDevicePanelStatus(activityPanel, status) { // this can be changed
  // This is the single place that turns an online/offline value into card text and styling.
  const isOnline = status === "live";

  activityPanel.panel.classList.toggle("is-offline", !isOnline);
  activityPanel.status.textContent = isOnline ? "LIVE" : "OFFLINE";
  activityPanel.status.classList.toggle("is-offline", !isOnline);
}

function setDeviceActivityHistoryOpen(activityPanel, isOpen, deviceId) {
  activityPanel.panel.classList.toggle("is-history-open", isOpen);
  activityPanel.toggle.setAttribute("aria-expanded", String(isOpen));
  activityPanel.toggle.setAttribute(
    "aria-label",
    `${isOpen ? "Hide" : "Show"} movement history for ${deviceId}`
  );
  activityPanel.activityList.setAttribute("aria-hidden", String(!isOpen));

  if (isOpen) {
    activityPanel.activityList.scrollTop = activityPanel.activityList.scrollHeight;
  }
}

function renderDeviceActivity(activityList, event) {
  const eventElement = document.createElement("li");
  const eventLabel = document.createElement("span");
  const eventDetail = document.createElement("span");

  eventElement.className = `device-activity__event device-activity__event--${event.eventType}`;
  eventLabel.className = "device-activity__event-label";
  eventDetail.className = "device-activity__event-detail";
  eventLabel.textContent = event.label;
  eventDetail.textContent = event.detail;
  eventElement.append(eventLabel, eventDetail);
  activityList.insertBefore(eventElement, activityList.firstElementChild);
}

function addDeviceActivity(activityPanel, key, label, detail, eventType) {
  const event = { label, detail, eventType };
  const activityHistory = deviceActivityHistory[key] || [];

  activityHistory.push(event);
  if (activityHistory.length > MAX_ACTIVITY_EVENTS_PER_DEVICE) {
    activityHistory.shift();
    activityPanel.activityList.firstElementChild?.remove();
  }

  deviceActivityHistory[key] = activityHistory;
  saveDeviceActivityHistory();
  renderDeviceActivity(activityPanel.activityList, event);

  if (activityPanel.panel.classList.contains("is-history-open")) {
    activityPanel.activityList.scrollTop = activityPanel.activityList.scrollHeight;
  }
}

// === Browser-saved position history ===

// The marker should disappear when a device goes offline, but its last known
// location must survive a brief missing JSON snapshot or Live Server reload.
const POSITION_HISTORY_STORAGE_KEY = "robot-room-last-positions";

function loadLastPositions() {
  try {
    const savedPositions = sessionStorage.getItem(POSITION_HISTORY_STORAGE_KEY);
    const parsedPositions = savedPositions ? JSON.parse(savedPositions) : {};

    return parsedPositions && typeof parsedPositions === "object" && !Array.isArray(parsedPositions)
      ? parsedPositions
      : {};
  } catch (error) {
    console.warn("Could not restore robot position history:", error);
    return {};
  }
}

function saveLastPositions() {
  sessionStorage.setItem(POSITION_HISTORY_STORAGE_KEY, JSON.stringify(lastPositions));
}

let lastPositions = loadLastPositions();

// === JSON polling and device connection state ===

async function fetchPositions() {
  try {
    const response = await fetch(`updated_pos.json?t=${Date.now()}`);
    const devices = await response.json();
    // Get current device keys from JSON
    const currentKeys = new Set();
    devices.forEach(device => {
      const key = `${device.id}-${device.device_type}`;
      currentKeys.add(key);

      const newPos = { x: device.pos.x, z: device.pos.z };
      const previousPos = lastPositions[key];
      const positionChanged = previousPos && (
        Math.abs(previousPos.x - newPos.x) > 0.01 ||
        Math.abs(previousPos.z - newPos.z) > 0.01
      );
      const { activityPanel, isNew } = getDeviceActivityPanel(device, key);

      // Today, appearing in updated_pos.json means LIVE. When server.py exports
      // device status, use that JSON value here instead of the fixed "live".
      setDevicePanelStatus(activityPanel, "live");
      activityPanel.latestPosition.textContent = `Latest: ${formatCoordinates(newPos)}`;

      // Update 3D marker
      updateRobotMarker(device);

      if (!previousPos) {
        addDeviceActivity(activityPanel, key, "Connected", formatCoordinates(newPos), "connected");
      } else if (positionChanged) {
        addDeviceActivity(
          activityPanel,
          key,
          "Previous",
          `${formatCoordinates(previousPos)}}`,
          "moved"
        );

        const dashboard = document.querySelector(".dashboard");
        dashboard.scrollTop = dashboard.scrollHeight;
      } else if (isNew && !deviceActivityHistory[key]?.length) {
        // origin point when device is first added
        addDeviceActivity(activityPanel, key, "Origin", formatCoordinates(newPos), "connected");
      }

      lastPositions[key] = newPos;
      saveLastPositions();
    });

    // A missing device is offline for this browser session: remove its map marker,
    // but keep its activity card and last-known position visible in the dashboard.
    Object.keys(robotMarkers).forEach(key => {
      if (!currentKeys.has(key)) {
        scene.remove(robotMarkers[key]);
        delete robotMarkers[key];

        if (deviceActivityPanels[key]) {
          setDevicePanelStatus(deviceActivityPanels[key], "offline");
        }
      }
    });
  } catch (error) {
    console.error("Error fetching positions:", error);
  }
}

// === Dashboard startup ===

// Poll latest exported device snapshot from updated_pos.json every 0.5s.
setInterval(fetchPositions, 500);
animate();
