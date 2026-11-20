/**
 * Room outline in dashboard-drawn meters. Keep in sync with map/room.py
 * (ROOM_CORNERS_DRAWN / DISPLAY_OFFSET) and dashboard/main.js roomCorners.
 */

// (0, 0) is the corner furthest from the door.
export const ROOM_CORNERS = [
  { x: 0, z: 0 },
  { x: 0, z: 15.45 },
  { x: 5.16, z: 15.45 },
  { x: 5.16, z: 12.26 },
  { x: 9.07, z: 12.26 },
  { x: 9.07, z: 0 },
];

// Matches map.room.DISPLAY_OFFSET (x, y, z). Grid cells are indexed in the
// server frame: drawn = server + (origin_x, origin_z).
export const DISPLAY_OFFSET = { x: 0.1, y: 0.1, z: 2.0 };

export const DEFAULT_CELL_SIZE = 0.1;

export function pointInRoomPolygon(x, z, corners = ROOM_CORNERS) {
  let inside = false;
  let prev = corners[corners.length - 1];
  for (const corner of corners) {
    if ((corner.z > z) !== (prev.z > z)) {
      const crossingX =
        ((prev.x - corner.x) * (z - corner.z)) / (prev.z - corner.z) + corner.x;
      if (x < crossingX) inside = !inside;
    }
    prev = corner;
  }
  return inside;
}
