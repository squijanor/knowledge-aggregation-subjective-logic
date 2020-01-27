from typing import List, NamedTuple, Union, Tuple, Optional
import bresenham
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid

from commons_msgs.msg import Goal


class Cell(NamedTuple):
    x: int
    y: int


class OccupancyMap:

    def __init__(self, grid: Optional[List[int]], origin: Point, width: int, height: int, resolution: float):
        self.resolution = resolution
        self.height = height
        self.width = width
        self.origin = origin
        self.grid = None if grid is None else list(grid)

    @staticmethod
    def from_message(message: OccupancyGrid, with_data=True) -> 'OccupancyMap':
        return OccupancyMap(message.data if with_data else [], message.info.origin.position, message.info.width,
                            message.info.height,
                            message.info.resolution)

    @staticmethod
    def make_direct_grid_path(map_start: Cell, map_goal: Cell) -> List[Cell]:
        return [Cell(x, y) for x, y in bresenham.bresenham(map_start.x, map_start.y, map_goal.x, map_goal.y)]

    def make_direct_grid_path_from_world(self, start: Point, goal: Point) -> List[Cell]:
        map_start = self.world2costmap(start)
        map_goal = self.world2costmap(goal)
        return [Cell(x, y) for x, y in bresenham.bresenham(map_start.x, map_start.y, map_goal.x, map_goal.y)]

    def is_obstacle_on_path(self, path: List[Cell]) -> Tuple[bool, int]:
        path_length: int = len(path)
        for i in range(path_length):
            check_position: int = self.to_costmap_index(path[i])
            value_in_costmap: int = self.grid[check_position]
            if value_in_costmap > 95:
                return True, i
        return False, len(path)

    def is_obstacle_on_world_position(self, world_point: Point) -> bool:
        if abs(world_point.x) > 4.5 or abs(world_point.y) > 4.5:
            return True
        return self.grid[self.to_costmap_index(self.world2costmap(world_point))] > 0

    def to_costmap_index(self, position: Cell) -> int:
        return position.y * self.width + position.x

    def from_costmap_index(self, index: int) -> Cell:
        return Cell(index % self.width, index // self.width)

    def world2costmap(self, world_coordinate: Union[Point, Goal]) -> Cell:
        return Cell(
            int(round((world_coordinate.x - self.origin.x) / self.resolution)),
            int(round((world_coordinate.y - self.origin.y) / self.resolution)))

    def trace_path_to_obstacle(self, center: Cell, edge: Cell):
        path = self.make_direct_grid_path(center, edge)
        return path[:self.is_obstacle_on_path(path)[1]]

    def costmap2world(self, cell: Cell) -> Point:
        return Point(cell.x * self.resolution + self.origin.x, cell.y * self.resolution + self.origin.y, 0)

    def to_message(self) -> OccupancyGrid:
        out_grid = OccupancyGrid()
        out_grid.info.origin = Pose(position=self.origin)
        out_grid.info.resolution = self.resolution
        out_grid.info.height = self.height
        out_grid.info.width = self.width
        out_grid.data = self.grid
        return out_grid
