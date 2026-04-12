#!/usr/bin/env python3

"""
planner_core.py

This file contains the map-processing and path-planning logic for the planner.

Main responsibilities:
1. Load the cave bitmap map
2. Convert the bitmap into a binary obstacle image
3. Build a coarse planning occupancy grid at 0.2 m resolution
4. Inflate obstacles for robot safety
5. Convert between world coordinates and grid coordinates
6. Plan a path using 8-connected A*
7. Prune the raw path using line-of-sight checks
8. Convert the final pruned path back into world coordinates

This file is intentionally kept independent from ROS so it is easier to test
and easier to explain in a viva.
"""

import math
import heapq
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image


class PlannerCore:
    """
    Core planner class containing map processing and A* path planning logic.
    """

    def __init__(
        self,
        map_image_path: str,
        map_origin: Tuple[float, float] = (-8.0, -8.0),
        map_resolution: float = 0.032,
        planning_resolution: float = 0.2,
        inflation_radius_m: float = 0.7
    ) -> None:
        """
        Initialize the planner core with map and planning settings.

        Args:
            map_image_path: path to cave_filled.png
            map_origin: world origin of the map in meters
            map_resolution: original bitmap map resolution in meters/pixel
            planning_resolution: coarse planning grid resolution in meters/cell
            inflation_radius_m: obstacle inflation radius in meters
        """
        self.map_image_path = map_image_path
        self.map_origin = map_origin
        self.map_resolution = map_resolution
        self.planning_resolution = planning_resolution
        self.inflation_radius_m = inflation_radius_m

        # Convert inflation radius from meters to planning-grid cells.
        self.inflation_radius_cells = int(
            math.ceil(self.inflation_radius_m / self.planning_resolution)
        )

        # These will be filled after prepare_map() is called.
        self.bitmap_image = None
        self.binary_image = None
        self.occupancy_grid = None
        self.inflated_grid = None

        self.image_height = 0
        self.image_width = 0
        self.grid_height = 0
        self.grid_width = 0

        self.map_prepared = False

    # ------------------------------------------------------------------
    # Map loading and preprocessing
    # ------------------------------------------------------------------

    def load_map_bitmap(self):
        """
        Load the bitmap map from disk and convert it to grayscale.

        Returns:
            Grayscale image as a NumPy array of shape (H, W)
        """
        image = Image.open(self.map_image_path).convert('L')
        image_array = np.array(image)

        self.bitmap_image = image_array
        self.image_height, self.image_width = image_array.shape
        return image_array

    def threshold_bitmap_to_binary(self, image):
        """
        Convert the grayscale bitmap into a binary obstacle image.

        Convention used here:
        - dark pixels   -> occupied (1)
        - bright pixels -> free (0)

        A simple threshold is used to keep the implementation clear.

        Args:
            image: grayscale image as NumPy array

        Returns:
            binary_image: NumPy array with values:
                1 = obstacle
                0 = free
        """
        # Conservative simple threshold:
        # anything darker than 128 is treated as an obstacle.
        binary_image = np.where(image < 128, 1, 0).astype(np.uint8)
        self.binary_image = binary_image
        return binary_image

    def build_planning_grid(self, binary_image):
        """
        Build a coarse occupancy grid at planning_resolution (0.2 m/cell).

        The original bitmap resolution is 0.032 m/pixel, so the planning grid
        is much coarser. We use a conservative occupancy rule:

        If ANY fine pixel inside a coarse planning cell is occupied,
        the entire coarse cell is marked occupied.

        This is safer and prevents thin obstacles from disappearing during
        downsampling.

        Args:
            binary_image: fine binary obstacle image (1 occupied, 0 free)

        Returns:
            occupancy_grid: coarse planning grid (1 occupied, 0 free)
        """
        map_width_m = self.image_width * self.map_resolution
        map_height_m = self.image_height * self.map_resolution

        self.grid_width = int(math.ceil(map_width_m / self.planning_resolution))
        self.grid_height = int(math.ceil(map_height_m / self.planning_resolution))

        occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)

        # Loop through every fine pixel. If it is occupied, mark the
        # corresponding coarse planning cell occupied.
        for row_px in range(self.image_height):
            for col_px in range(self.image_width):
                if binary_image[row_px, col_px] == 1:
                    coarse_col = int((col_px * self.map_resolution) / self.planning_resolution)

                    row_from_bottom = int(
                        ((self.image_height - 1 - row_px) * self.map_resolution) / self.planning_resolution
                    )
                    coarse_row = self.grid_height - 1 - row_from_bottom

                    if 0 <= coarse_row < self.grid_height and 0 <= coarse_col < self.grid_width:
                        occupancy_grid[coarse_row, coarse_col] = 1

        self.occupancy_grid = occupancy_grid
        return occupancy_grid

    def inflate_obstacles(self, occupancy_grid):
        """
        Inflate obstacles using the given inflation radius.

        Every occupied cell expands to nearby cells within the inflation radius.
        This creates a safer planning grid with buffer space around obstacles.

        Args:
            occupancy_grid: coarse occupancy grid (1 occupied, 0 free)

        Returns:
            inflated_grid: inflated occupancy grid
        """
        inflated_grid = occupancy_grid.copy()
        occupied_cells = np.argwhere(occupancy_grid == 1)

        for row, col in occupied_cells:
            for d_row in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                for d_col in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                    new_row = row + d_row
                    new_col = col + d_col

                    if not (0 <= new_row < self.grid_height and 0 <= new_col < self.grid_width):
                        continue

                    distance_cells = math.sqrt(d_row ** 2 + d_col ** 2)
                    if distance_cells <= self.inflation_radius_cells:
                        inflated_grid[new_row, new_col] = 1

        self.inflated_grid = inflated_grid
        return inflated_grid

    def prepare_map(self) -> None:
        """
        Run the full map preparation pipeline once.

        Steps:
        1. Load bitmap image
        2. Threshold to binary obstacle image
        3. Build coarse planning occupancy grid
        4. Inflate obstacles
        """
        image = self.load_map_bitmap()
        binary_image = self.threshold_bitmap_to_binary(image)
        occupancy_grid = self.build_planning_grid(binary_image)
        self.inflate_obstacles(occupancy_grid)
        self.map_prepared = True

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates (meters) into planning-grid coordinates.

        World frame:
        - origin is at (-8.0, -8.0)
        - x increases to the right
        - y increases upward

        Grid frame:
        - row increases downward
        - col increases to the right

        Args:
            x: world x coordinate
            y: world y coordinate

        Returns:
            (row, col) in planning grid
        """
        col = int(math.floor((x - self.map_origin[0]) / self.planning_resolution))

        row_from_bottom = int(math.floor((y - self.map_origin[1]) / self.planning_resolution))
        row = self.grid_height - 1 - row_from_bottom

        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert planning-grid coordinates into the world position of the
        center of that cell.

        Args:
            row: grid row
            col: grid col

        Returns:
            (x, y) world coordinates at the center of the cell
        """
        x = self.map_origin[0] + (col + 0.5) * self.planning_resolution

        row_from_bottom = self.grid_height - 1 - row
        y = self.map_origin[1] + (row_from_bottom + 0.5) * self.planning_resolution

        return x, y

    # ------------------------------------------------------------------
    # Grid checks
    # ------------------------------------------------------------------

    def is_in_bounds(self, cell: Tuple[int, int], grid) -> bool:
        """
        Check if a cell is inside the grid bounds.
        """
        row, col = cell
        return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]

    def is_occupied(self, cell: Tuple[int, int], grid) -> bool:
        """
        Check if a cell is occupied.
        """
        row, col = cell
        return grid[row, col] == 1

    def is_free(self, cell: Tuple[int, int], grid) -> bool:
        """
        Check if a cell is free.
        """
        return self.is_in_bounds(cell, grid) and not self.is_occupied(cell, grid)

    # ------------------------------------------------------------------
    # A* path planning
    # ------------------------------------------------------------------

    def heuristic(self, cell: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """
        Euclidean heuristic for A*.

        Args:
            cell: current cell
            goal: goal cell

        Returns:
            Euclidean distance between the two cells
        """
        return math.sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)

    def get_neighbors(self, cell: Tuple[int, int], grid) -> List[Tuple[Tuple[int, int], float]]:
        """
        Return valid 8-connected neighbors and their movement costs.

        Straight moves cost 1.0
        Diagonal moves cost sqrt(2)

        Args:
            cell: current cell
            grid: occupancy grid

        Returns:
            list of ((neighbor_row, neighbor_col), move_cost)
        """
        row, col = cell

        neighbor_offsets = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)),
            ( 1,  1, math.sqrt(2)),
        ]

        neighbors = []

        for d_row, d_col, cost in neighbor_offsets:
            new_cell = (row + d_row, col + d_col)

            if not self.is_free(new_cell, grid):
                continue

            if abs(d_row) == 1 and abs(d_col) == 1:
                side_a = (row + d_row, col)
                side_b = (row, col + d_col)
                if not self.is_free(side_a, grid) or not self.is_free(side_b, grid):
                    continue

            neighbors.append((new_cell, cost))

        return neighbors

    def reconstruct_path(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        goal_cell: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        Reconstruct the A* path from the came_from dictionary.

        Args:
            came_from: parent mapping for each visited node
            goal_cell: goal cell

        Returns:
            ordered path from start to goal
        """
        path = [goal_cell]
        current = goal_cell

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def astar_search(
        self,
        start_cell: Tuple[int, int],
        goal_cell: Tuple[int, int],
        grid
    ) -> List[Tuple[int, int]]:
        """
        Run A* on the occupancy grid.

        Uses:
        - 8-connected motion
        - Euclidean heuristic

        Args:
            start_cell: starting cell
            goal_cell: goal cell
            grid: occupancy grid to search on

        Returns:
            path as list of grid cells from start to goal,
            or empty list if no path is found
        """
        if not self.is_free(start_cell, grid):
            return []

        if not self.is_free(goal_cell, grid):
            return []

        open_heap = []
        heapq.heappush(open_heap, (0.0, start_cell))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_costs: Dict[Tuple[int, int], float] = {start_cell: 0.0}

        closed_set = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed_set:
                continue

            if current == goal_cell:
                return self.reconstruct_path(came_from, goal_cell)

            closed_set.add(current)

            for neighbor, move_cost in self.get_neighbors(current, grid):
                if neighbor in closed_set:
                    continue

                tentative_g = g_costs[current] + move_cost

                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    priority = tentative_g + self.heuristic(neighbor, goal_cell)
                    came_from[neighbor] = current
                    heapq.heappush(open_heap, (priority, neighbor))

        return []

    # ------------------------------------------------------------------
    # Path pruning
    # ------------------------------------------------------------------

    def line_of_sight(
        self,
        cell_a: Tuple[int, int],
        cell_b: Tuple[int, int],
        grid
    ) -> bool:
        """
        Check whether two grid cells can be connected by a straight line
        without passing through occupied cells.

        A simple Bresenham-style integer grid traversal is used.

        Args:
            cell_a: first cell
            cell_b: second cell
            grid: occupancy grid

        Returns:
            True if the straight line is collision-free, False otherwise
        """
        row0, col0 = cell_a
        row1, col1 = cell_b

        d_row = abs(row1 - row0)
        d_col = abs(col1 - col0)

        step_row = 1 if row0 < row1 else -1
        step_col = 1 if col0 < col1 else -1

        err = d_col - d_row

        current_row = row0
        current_col = col0

        while True:
            current_cell = (current_row, current_col)

            if not self.is_in_bounds(current_cell, grid):
                return False

            if self.is_occupied(current_cell, grid):
                return False

            if current_row == row1 and current_col == col1:
                break

            e2 = 2 * err

            if e2 > -d_row:
                err -= d_row
                current_col += step_col

            if e2 < d_col:
                err += d_col
                current_row += step_row

        return True

    def prune_path(
        self,
        path_cells: List[Tuple[int, int]],
        grid
    ) -> List[Tuple[int, int]]:
        """
        Prune the raw path using line-of-sight checks.

        Idea:
        - keep the first point
        - try to jump as far ahead as possible
        - if direct line-of-sight exists, skip intermediate points

        Args:
            path_cells: raw A* path
            grid: occupancy grid used for collision checks

        Returns:
            pruned path
        """
        if not path_cells:
            return []

        if len(path_cells) <= 2:
            return path_cells.copy()

        pruned_path = [path_cells[0]]
        current_index = 0

        while current_index < len(path_cells) - 1:
            next_index = len(path_cells) - 1

            # Try to connect current point directly to the farthest possible point.
            while next_index > current_index + 1:
                if self.line_of_sight(path_cells[current_index], path_cells[next_index], grid):
                    break
                next_index -= 1

            pruned_path.append(path_cells[next_index])
            current_index = next_index

        return pruned_path

    def cells_to_world_path(
        self,
        path_cells: List[Tuple[int, int]]
    ) -> List[Tuple[float, float]]:
        """
        Convert a list of grid cells into world-coordinate waypoints.

        Args:
            path_cells: path in grid coordinates

        Returns:
            path in world coordinates
        """
        world_path = []
        for row, col in path_cells:
            world_path.append(self.grid_to_world(row, col))
        return world_path