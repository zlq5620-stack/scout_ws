"""ROS-independent, conservative occupancy-grid A* search."""
import heapq
import math
import numpy as np
from scipy.ndimage import binary_dilation


class GridPlanner:
    def __init__(self, values, resolution, origin, clearance_radius=.47):
        values = np.asarray(values)
        if values.ndim != 2 or not values.size or values.size > 2000000:
            raise ValueError('Map must be a nonempty 2D array with <= 2M cells')
        if not np.all(np.isfinite(values)) or np.any((values < -1) | (values > 100)):
            raise ValueError('Invalid occupancy values')
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid map resolution')
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
            raise ValueError('Invalid map origin')
        if not math.isfinite(clearance_radius) or not 0 < clearance_radius <= 2:
            raise ValueError('clearance_radius must be in (0, 2] m')
        self.values = values.copy()
        self.resolution = float(resolution)
        self.origin = tuple(origin)
        self.clearance_radius = float(clearance_radius)
        # Account conservatively for obstacle cell area and endpoint subcell offset.
        self.inflation_cells = int(math.ceil(clearance_radius / resolution + math.sqrt(2)))
        if self.inflation_cells > 1000:
            raise ValueError('Inflation kernel too large; check resolution/radius')
        n = self.inflation_cells
        yy, xx = np.ogrid[-n:n+1, -n:n+1]
        disk = xx * xx + yy * yy <= n * n
        occupied = (values < 0) | (values >= 60)
        self.blocked = binary_dilation(occupied, structure=disk, border_value=1)

    def world_to_cell(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        u = (math.cos(yaw) * dx + math.sin(yaw) * dy) / self.resolution
        v = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / self.resolution
        return int(math.floor(u)), int(math.floor(v))

    def cell_to_world(self, i, j):
        ox, oy, yaw = self.origin
        u, v = (i + .5) * self.resolution, (j + .5) * self.resolution
        return ox + math.cos(yaw) * u - math.sin(yaw) * v, oy + math.sin(yaw) * u + math.cos(yaw) * v

    def usable(self, cell):
        x, y = cell
        return 0 <= x < self.blocked.shape[1] and 0 <= y < self.blocked.shape[0] and not self.blocked[y, x]

    def search(self, start, goal, max_expansions=250000):
        """Return (cell path or None, status, expanded count), with no corner cuts."""
        if not self.usable(start):
            return None, 'START_BLOCKED', 0
        if not self.usable(goal):
            return None, 'GOAL_BLOCKED', 0
        if max_expansions <= 0:
            return None, 'EXPANSION_LIMIT', 0
        def heuristic(cell):
            dx, dy = abs(goal[0] - cell[0]), abs(goal[1] - cell[1])
            return dx + dy + (math.sqrt(2) - 2) * min(dx, dy)
        offsets = [(dx, dy, math.hypot(dx, dy)) for dx, dy in
                   ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))]
        heap = [(heuristic(start), start)]
        costs = {start: 0.}
        parents = {}
        visited = set()
        while heap and len(visited) < max_expansions:
            _, current = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                path = [current]
                while current in parents:
                    current = parents[current]
                    path.append(current)
                return path[::-1], 'PATH_READY', len(visited)
            for dx, dy, step in offsets:
                nxt = current[0] + dx, current[1] + dy
                if not self.usable(nxt):
                    continue
                if dx and dy and (not self.usable((current[0] + dx, current[1])) or
                                  not self.usable((current[0], current[1] + dy))):
                    continue
                score = costs[current] + step
                if score < costs.get(nxt, float('inf')):
                    costs[nxt] = score
                    parents[nxt] = current
                    heapq.heappush(heap, (score + heuristic(nxt), nxt))
        return None, 'EXPANSION_LIMIT' if heap else 'NO_PATH', len(visited)

    def inflated_values(self):
        result = np.where(self.blocked, 100, 0).astype(np.int8)
        result[self.values < 0] = -1
        return result
