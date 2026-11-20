import numpy as np
from map.wallgrid import WallGrid
# Used to localize robot against the room

def precompute_max_grids(grid: np.ndarray, levels: int) -> list[np.ndarray]:
    stack = [grid]
    for h in range(1, levels):
        prev, w = stack[-1], 2 ** (h - 1)
        nxt = prev.copy()
        nxt[:-w, :] = np.maximum(prev[:-w, :], prev[w:, :])
        nxt[:, :-w] = np.maximum(nxt[:, :-w], nxt[:, w:])
        stack.append(nxt)
    return stack

class BranchAndBound:
    def __init__(self, wall_grid: WallGrid, levels: int = 7):
        self.wall_grid = wall_grid

        # Required padding for out of range scans to not destroy measurements
        P = 300
        self.pad = P
        self.stack = precompute_max_grids(np.pad(wall_grid.likelihood, P, constant_values=0.0), levels)        

    def search(self, scan_rel, center, xy_window, theta_window, min_score):
        # Thin the scan for dupes
        res = self.wall_grid.resolution
        _, keep = np.unique(np.floor(scan_rel/self.wall_grid.resolution).astype(int), axis=0, return_index=True)
        scan_clean = scan_rel[keep]

        d_max = np.hypot(scan_clean[:, 0], scan_clean[:, 1]).max()

        d_theta = np.arccos(1 - res**2 / (2 * d_max**2))
        n = int(np.ceil(theta_window / d_theta))
        thetas = center[2] + np.arange(-n, n + 1) * d_theta
        offsets = []
        for t in thetas:
            sin_t, cos_t = np.sin(t), np.cos(t)
            # rotation array here
            rotated = scan_clean @ np.array([[cos_t, -sin_t], [sin_t, cos_t]]).T
            # Convert from scan to grid cell and save 
            offsets.append(np.round(rotated / res).astype(int)) # Offsets[0] uses thetas[0], offsets[1] w thetas[1] , etc


        origin = self.wall_grid.origin
        cx = int(np.floor((center[0] - origin[0]) / res))   # center's cell
        cz = int(np.floor((center[1] - origin[1]) / res))
        wx = int(np.ceil(xy_window / res))
        x_lo, x_hi = cx - wx, cx + wx                         # inclusive bounds on robot cell
        z_lo, z_hi = cz - wx, cz + wx

        # Score over current H
        H = len(self.stack) - 1
        step = 2 ** H
        roots = []
        for a, off in enumerate(offsets):
            for x0 in range(x_lo, x_hi + 1, step):
                for z0 in range(z_lo, z_hi + 1, step):
                    roots.append((self.score(H, off, x0, z0), a, x0, z0, H))

        # starting with best root here for dfs
        roots.sort()
        todo = roots
        best_score, best = min_score, None

        while todo:
            s, a, x0, z0, h = todo.pop()

            # s is the best score for the node, nothing within it can be better, so we prune if potential cannot beat max score
            if s <= best_score:
                continue
            if h == 0:
                best_score, best = s, (a, x0, z0)   # exact score, a new best
                continue

            # Basically, this part searches the next sample down (h - 1) over the four children,
            # and then only adds the ones with better scores to the todo list
            half = 2 ** (h - 1)
            children = []
            # 0, half is a tuple, so only four samples searched here
            for dx in (0, half):
                for dz in (0, half):
                    cx0, cz0 = x0 + dx, z0 + dz
                    if cx0 > x_hi or cz0 > z_hi:
                        continue             # outside the window
                    cs = self.score(h - 1, offsets[a], cx0, cz0)
                    if cs > best_score:
                        children.append((cs, a, cx0, cz0, h - 1))
            children.sort()
            todo.extend(children)            # the best child is popped next
        if best is None:
            return None

        a, x0, z0 = best
        x = origin[0] + (x0 + 0.5) * res
        z = origin[1] + (z0 + 0.5) * res
        return x, z, thetas[a], best_score




        # essentially, since the points indicate hitting a "wall", we get the closeness
        # information from compressed grid of level h and see how well it matches up
    def score(self, h, offsets, x0, z0):
        idx = offsets + (x0 + self.pad, z0 + self.pad)
        return self.stack[h][idx[:, 0], idx[:, 1]].mean()








    
