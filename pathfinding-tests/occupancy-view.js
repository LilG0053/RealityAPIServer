/**
 * Occupancy lattice + probability coloring, adapted from dashboard/main.js.
 * Builds the room cell grid once, then paints cells from occupancy_grid.json.
 */

import * as THREE from "three";
import {
  DEFAULT_CELL_SIZE,
  DISPLAY_OFFSET,
  ROOM_CORNERS,
  pointInRoomPolygon,
} from "./room.js";

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

export function ensureOccupancyLattice(
  scene,
  cellSize = DEFAULT_CELL_SIZE,
  originX = DISPLAY_OFFSET.x,
  originZ = DISPLAY_OFFSET.z,
) {
  const key = `${cellSize}:${originX}:${originZ}`;
  if (occupancyMesh && occupancyLayoutKey === key) return occupancyMesh;

  if (occupancyMesh) {
    scene.remove(occupancyMesh);
    occupancyMesh.dispose();
  }
  occupancyLayoutKey = key;
  occupancyIndexByKey = new Map();

  const serverCorners = ROOM_CORNERS.map((c) => ({
    x: c.x - originX,
    z: c.z - originZ,
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

  occupancyMesh = new THREE.InstancedMesh(
    occupancyGeometry,
    occupancyMaterial,
    coords.length,
  );
  occupancyMarker.rotation.set(-Math.PI / 2, 0, 0);
  occupancyMarker.scale.set(cellSize * 0.96, cellSize * 0.96, 1);

  coords.forEach(([cx, cz], index) => {
    occupancyIndexByKey.set(`${cx},${cz}`, index);
    occupancyMarker.position.set(
      (cx + 0.5) * cellSize + originX,
      0.015,
      (cz + 0.5) * cellSize + originZ,
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
  return occupancyMesh;
}

export function updateOccupancyGrid(scene, grid) {
  if (!Number.isFinite(grid.cell_size) || grid.cell_size <= 0 || !Array.isArray(grid.cells)) {
    return;
  }
  const originX = Number.isFinite(grid.origin_x) ? grid.origin_x : DISPLAY_OFFSET.x;
  const originZ = Number.isFinite(grid.origin_z) ? grid.origin_z : DISPLAY_OFFSET.z;
  ensureOccupancyLattice(scene, grid.cell_size, originX, originZ);
  if (!occupancyMesh) return;

  if (grid.reset) {
    for (let index = 0; index < occupancyMesh.count; index += 1) {
      occupancyMesh.setColorAt(index, unknownColor);
    }
  }

  for (const cell of grid.cells) {
    const index = occupancyIndexByKey.get(`${cell.x},${cell.z}`);
    if (index === undefined) continue;
    if (!Number.isFinite(cell.p)) continue;
    occupancyMesh.setColorAt(index, colorForProbability(cell.p));
  }

  if (occupancyMesh.instanceColor) occupancyMesh.instanceColor.needsUpdate = true;
}

export function getOccupancyIndexByKey() {
  return occupancyIndexByKey;
}
