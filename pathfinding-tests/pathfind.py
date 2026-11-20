"""Quick pathfinding helpers. Run from the repo root:

    .venv/bin/python pathfinding-tests/pathfind.py
"""

from __future__ import annotations

from collections import deque
import heapq
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from map.occupancy import CellIndex, OccupancyGrid

# 4-connected + diagonals, in cell-index steps (dx, dz).
NEIGHBOR_STEPS = (
    (1, 0), (-1, 0),
    (0, 1), (0, -1),
    (1, 1), (1, -1),
    (-1, 1), (-1, -1),
)

# Step tuple → travel cost in cell units (orthogonal = 1, diagonal = √2).
STEP_COSTS = {
    step: math.sqrt(step[0] ** 2 + step[1] ** 2) for step in NEIGHBOR_STEPS
}

def prob_of_cell(grid: OccupancyGrid, cell: CellIndex): return OccupancyGrid.cell()

def get_neighbors_with_costs(
    grid: OccupancyGrid,
    cell: CellIndex,
) -> dict[CellIndex, float]:
    """Return in-room neighbors of ``cell`` mapped to their step costs."""
    x, z = cell
    neighbors_with_costs: dict[CellIndex, float] = {}
    for dx, dz in NEIGHBOR_STEPS:
        neighbor: CellIndex = (x + dx, z + dz)
        if neighbor in grid.room_cells:
            neighbors_with_costs[neighbor] = STEP_COSTS[(dx, dz)]
    return neighbors_with_costs

def dijkstra(grid: OccupancyGrid, start: CellIndex, goal: CellIndex):
    # Priority queue of (cost_from_start, cell). Lowest cost is popped first.
    frontier = [(0, start)]
    parents = { cell : None for cell in grid.room_cells() }

    visited = {start}

    # Best known cost from start to each room cell (inf = unreachable so far).
    best_cost = {
        cell: math.inf if cell != start else 0
        for cell in grid.room_cells
    }

    while len(frontier) and (current := heapq.heappop(frontier)) != goal:
        cost_from_start, current_cell = current

        for neighbor, step_cost in get_neighbors_with_costs(grid, current_cell):
            visited.add(neighbor)

            candidate_cost = cost_from_start + step_cost
            if candidate_cost < best_cost[neighbor]:
                best_cost[neighbor] = candidate_cost
                parents[best_cost] = current_cell
                heapq.heappush(frontier, (candidate_cost, neighbor))

    if parents[goal]: 
        path = []

        curr = goal
        while curr is not None: 
            path.append(curr)
        print(curr )

def main() -> None:
    grid = OccupancyGrid()
    # Pick a cell that is actually inside the room lattice.
    start = next(iter(grid.room_cells))
    neighbors = get_neighbors_with_costs(grid, start)
    print(f"start={start}")
    print(f"neighbor_count={len(neighbors)}")
    for neighbor, cost in sorted(neighbors.items()):
        print(f"  {neighbor}  cost={cost:.4f}")

if __name__ == "__main__":
    main()
