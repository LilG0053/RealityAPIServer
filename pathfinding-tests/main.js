/**
 * Temporary pathfinding test view: room + occupancy grid only.
 * Serve the repo root so this page can fetch ../dashboard/occupancy_grid.json:
 *   python -m http.server 8000
 * then open http://localhost:8000/pathfinding-tests/
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ROOM_CORNERS } from "./room.js";
import { ensureOccupancyLattice, updateOccupancyGrid } from "./occupancy-view.js";

const OCCUPANCY_URL = "../dashboard/occupancy_grid.json";
const WALL_HEIGHT = 3;
const WALL_THICKNESS = 0.15;

const sceneContainer = document.getElementById("scene-container");
const statusEl = document.getElementById("occupancy-status");

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x20232a);
// Mirror so positive sensor Z matches the physical room layout.
scene.scale.z = -1;

const camera = new THREE.PerspectiveCamera(
  55,
  sceneContainer.clientWidth / sceneContainer.clientHeight,
  0.1,
  1000,
);
camera.position.set(-3, 15, 10);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(sceneContainer.clientWidth, sceneContainer.clientHeight);
sceneContainer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.screenSpacePanning = true;
controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
controls.minPolarAngle = Math.PI / 6;
controls.maxPolarAngle = Math.PI / 2.3;
controls.minDistance = 2;
controls.maxDistance = 30;

scene.add(new THREE.HemisphereLight(0xffffff, 0x2b3340, 2));
const directionalLight = new THREE.DirectionalLight(0xffffff, 2);
directionalLight.position.set(5, 10, 5);
scene.add(directionalLight);

const floorShape = new THREE.Shape();
floorShape.moveTo(ROOM_CORNERS[0].x, ROOM_CORNERS[0].z);
for (const corner of ROOM_CORNERS.slice(1)) {
  floorShape.lineTo(corner.x, corner.z);
}
floorShape.closePath();
const floor = new THREE.Mesh(
  new THREE.ShapeGeometry(floorShape),
  new THREE.MeshBasicMaterial({ color: 0x68717d, side: THREE.DoubleSide }),
);
floor.rotation.x = Math.PI / 2;
scene.add(floor);

scene.add(new THREE.AxesHelper(2));

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
      side: THREE.DoubleSide,
    }),
  );
  wall.position.set(
    (start.x + end.x) / 2,
    WALL_HEIGHT / 2,
    (start.z + end.z) / 2,
  );
  wall.rotation.y = Math.atan2(-dz, dx);
  scene.add(wall);
}

for (let index = 0; index < ROOM_CORNERS.length; index += 1) {
  createWallBetween(
    ROOM_CORNERS[index],
    ROOM_CORNERS[(index + 1) % ROOM_CORNERS.length],
  );
}

scene.add(
  new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(
      ROOM_CORNERS.map((corner) => new THREE.Vector3(corner.x, 0.02, corner.z)),
    ),
    new THREE.LineBasicMaterial({ color: 0xffffff }),
  ),
);

// Empty lattice so pathfinding work has a grid even before occupancy JSON exists.
ensureOccupancyLattice(scene);

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function pollOccupancyGrid() {
  while (true) {
    try {
      const response = await fetch(OCCUPANCY_URL, { cache: "no-store" });
      if (response.ok) {
        updateOccupancyGrid(scene, await response.json());
        if (statusEl) statusEl.textContent = "Occupancy grid loaded";
      } else {
        if (statusEl) statusEl.textContent = "Waiting for occupancy grid";
        if (response.status !== 404) {
          console.error("Could not load occupancy grid:", response.status);
        }
      }
    } catch (error) {
      if (statusEl) statusEl.textContent = "Occupancy grid unavailable";
      console.error("Error loading occupancy grid:", error);
    }
    await sleep(1000);
  }
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

window.addEventListener("resize", () => {
  camera.aspect = sceneContainer.clientWidth / sceneContainer.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(sceneContainer.clientWidth, sceneContainer.clientHeight);
});

pollOccupancyGrid();
animate();
