"""
Control script that publishes path commands to trajectory following node.
"""
from typing import Optional, List, Dict, Sequence
import enum

import networkx as nx
import numpy as np

from optimal_navigation.optimal_navigation.optimal_navigation_to_center import main


class Direction(enum.Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


class RobotAction(enum.Enum):
    TURN_LEFT = enum.auto()
    TURN_RIGHT = enum.auto()
    MOVE_FORWARD = enum.auto()
    MOVE_BACK = enum.auto()


class DFSNodeStatus(enum.Enum):
    OPEN = enum.auto()
    VISITED = enum.auto()
    UNKNOWN = enum.auto()


class Cell:
    """
    Grid cell
    """

    def __init__(self, row: int, col: int):
        """

        :param row: row number, from top to bottom
        :param col: column number, from left to right
        """
        self.row = row
        self.col = col

        # None = unknown, false/true wall was detected / not detected in that direction
        self.walls: Dict[Direction, Optional[bool]] = {
            Direction.NORTH: None,
            Direction.SOUTH: None,
            Direction.WEST: None,
            Direction.EAST: None,
        }

    def set_walls(self, wall_list: Sequence[Direction]):
        # Init with false since all walls will be added, the rest are empty spaces
        for key in self.walls:
            self.walls[key] = False

        for wall in wall_list:
            self.walls[wall] = True

    def get_wall_coding(self) -> int:
        """
        According to problem1 rules.
        """
        assert not any(val is None for val in self.walls.values()), f"Can't get wall coding for cell {self} since not" \
                                                                    f" fully explored"
        if not self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and not self.walls[
            Direction.WEST] and not self.walls[Direction.EAST]:
            return 0
        if not self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 1
        if self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 2
        if not self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 3
        if not self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 4
        if not self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 5
        if not self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 6
        if self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 7
        if self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 8
        if not self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 9
        if self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 10
        if self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and not self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 11
        if self.walls[Direction.NORTH] and not self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 12
        if self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and not \
                self.walls[Direction.EAST]:
            return 13
        if not self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and \
                self.walls[Direction.EAST]:
            return 14
        if self.walls[Direction.NORTH] and self.walls[Direction.SOUTH] and self.walls[Direction.WEST] and self.walls[
            Direction.EAST]:
            return 15

    def __repr__(self):
        return f"({self.row:2};{self.col:2})"

    def is_adjacent_to(self, other_cell):
        if self.row == other_cell.row and self.col == other_cell.col:
            print("WARNING: testing adjacency to oneself")
            return True

        # row adjacent
        if self.row == other_cell.row and (self.col == other_cell.col + 1 or self.col == other_cell.col - 1):
            return True

        # column adjacent
        if self.col == other_cell.col and (self.row == other_cell.row + 1 or self.row == other_cell.row - 1):
            return True

        return False


class Pose:
    """
    Robot pose: (x,y) or cell and orientation (angle or discrete N-S-W-E).
    """

    def __init__(self, cell: Cell, orientation: Direction):
        self.cell = cell
        self.orientation = orientation

    def __repr__(self):
        return f"cell {self.cell} orientation {self.orientation.name}"

    def get_abs_front_orientation(self) -> Direction:
        """
        What absolute orientation is for the "front" direction of the agent
        """
        return self.orientation

    def get_abs_back_orientation(self) -> Direction:
        """
        What absolute orientation is for the "back" direction of the agent
        """
        if self.orientation == Direction.NORTH:
            return Direction.SOUTH
        if self.orientation == Direction.SOUTH:
            return Direction.NORTH
        if self.orientation == Direction.EAST:
            return Direction.WEST
        if self.orientation == Direction.WEST:
            return Direction.EAST

    def get_abs_left_orientation(self) -> Direction:
        """
        What absolute orientation is for the "left" direction of the agent
        """
        if self.orientation == Direction.NORTH:
            return Direction.WEST
        if self.orientation == Direction.SOUTH:
            return Direction.EAST
        if self.orientation == Direction.EAST:
            return Direction.NORTH
        if self.orientation == Direction.WEST:
            return Direction.SOUTH

    def get_abs_right_orientation(self) -> Direction:
        """
        What absolute orientation is for the "right" direction of the agent
        """
        if self.orientation == Direction.NORTH:
            return Direction.EAST
        if self.orientation == Direction.SOUTH:
            return Direction.WEST
        if self.orientation == Direction.EAST:
            return Direction.SOUTH
        if self.orientation == Direction.WEST:
            return Direction.NORTH
        raise AttributeError("Orientation is none of the cardinal directions")


class Map:
    """
    Top-left corner is 0,0
    """

    def __init__(self):
        # TODO: add an option to treat unknown cells on map as free (for planning in unknown space)
        self.n_rows = 16
        self.n_cols = 16

        self.cells = []
        for row in range(self.n_rows):
            row_arr = []
            for col in range(self.n_cols):
                row_arr.append(Cell(row, col))
            self.cells.append(row_arr)

    def __getitem__(self, index):
        return self.cells[index]

    def get_solution_map(self):
        solution_map = []
        for row in range(self.n_rows):
            row_arr = []
            for col in range(self.n_cols):
                wall_coding = self.cells[row][col].get_wall_coding()
                row_arr.append(wall_coding)
            solution_map.append(row_arr)

        return solution_map

    def __repr__(self):
        outstr = ""
        for row in self.cells:
            row_str = ""
            for cell in row:
                row_str += str(cell) + ' '
            outstr += row_str + "\n"

        return outstr

    def get_neighbours(self, cell: Cell) -> Sequence[Cell]:
        """
        Add neighbouring cells (no diagonals) if not outside of boundaries
        and no walls detected. Can't add for unexplored cell.

        :param cell:
        :return:
        """
        assert (
            not any(val is None for val in cell.walls.values())), "cell must be explored before determining neighbours"

        neighbours = []
        if not cell.walls[Direction.NORTH] and cell.row - 1 >= 0:
            neighbours.append(self.cells[cell.row - 1][cell.col])

        if not cell.walls[Direction.SOUTH] and cell.row + 1 < self.n_rows:
            neighbours.append(self.cells[cell.row + 1][cell.col])

        if not cell.walls[Direction.WEST] and cell.col - 1 >= 0:
            neighbours.append(self.cells[cell.row][cell.col - 1])

        if not cell.walls[Direction.EAST] and cell.col + 1 < self.n_cols:
            neighbours.append(self.cells[cell.row][cell.col + 1])

        return neighbours

    def update(self, cell: Cell, walls: Sequence[Direction]):
        """
        Updates the map with data from the given pose
        :param cell: Where to update the map
        :param walls: Walls info around the cell
        """
        self.cells[cell.row][cell.col].set_walls(walls)

    def is_full(self):
        """
        Checks if all map has been explored
        :return: True if there are no unexplored cells, False otherwise
        """
        all_walls = (wall_val for row in self.cells for cell in row for wall_val in cell.walls.values())
        return all(val is not None for val in all_walls)


def get_map_from_service() -> Map:
    # TODO: get from slam node
    return Map()


class MapSearchNode:
    """
    A node for networkx graph. Pathfinding and other operations will be done using this.
    The base structure underneath is the grid Map consisting of cells and walls around them
    that define directly accessible neighbours. Generally, only north, south, east and west
    movement is allowed.
    """

    def __init__(self, cell: Cell):
        self.cell = cell

    def __hash__(self) -> int:
        return hash((self.cell.col, self.cell.row))

    def __eq__(self, other) -> bool:
        """
        Compares map nodes (for hashability, so uses the same variables to compare).
        :param other: MapSearchNode to compare to
        :return:
        """
        return self.cell.col == other.cell.col and self.cell.row == other.cell.row

    def __repr__(self) -> str:
        return str(self.cell)


def cell_traversal_heuristic(node1: MapSearchNode, node2: MapSearchNode) -> float:
    """
    Heuristic function for search nodes based on grid map cells.
    """
    node1_coords = np.array((node1.cell.row, node1.cell.col))
    node2_coords = np.array((node2.cell.row, node2.cell.col))

    return float(np.linalg.norm(node1_coords - node2_coords))


class NavigationAgent:
    """
    Uses a map of the environment to build optimal plans from one point
    to another and move along these plans (paths).
    """

    def __init__(
            self, map_: Map
    ):
        self.map = map_

        # plan should connect current pose to the goal pose
        self.planned_path: Optional[List[Pose]] = None

    def plan(self, pose_from: Pose, pose_to: Pose) -> List[Pose]:
        """
        Builds a plan from pose_from to pose_to. It starts from the pose_from
        and goes through adjacent cells to the pose_to.
        """
        if self.map is None:
            raise AttributeError("Can't plan a path: no map given")

        search_graph = nx.Graph()
        edges = []
        for row_idx in range(self.map.n_rows):
            for col_idx in range(self.map.n_cols):
                cell = self.map[row_idx][col_idx]
                for neighbour in self.map.get_neighbours(cell):
                    # node, node, weight
                    edges.append((MapSearchNode(cell), MapSearchNode(neighbour), 1))
        search_graph.add_weighted_edges_from(edges)

        starting_node = MapSearchNode(pose_from.cell)
        goal_node = MapSearchNode(pose_to.cell)
        # Example usage of astar_path
        planned_path_nodes = nx.astar_path(search_graph, source=starting_node, target=goal_node,
                                           heuristic=cell_traversal_heuristic,
                                           weight='weight')

        # change search nodes path to poses
        self.planned_path = [Pose(node.cell, Direction.NORTH) for node in planned_path_nodes]

        return self.planned_path


class DepthFirstExploration:
    class Node:
        """
        DFS node with some extra info for cells.
        """

        def __init__(self, cell: Cell, parent_cell: Optional[Cell] = None,
                     node_status: Optional[
                         DFSNodeStatus] = DFSNodeStatus.UNKNOWN):
            """
            :param cell: current node's cell info (coordinates and whatever other info is there)
            :param parent_cell: cell on the path to root of the search tree (starting cell)
            """
            self.cell = cell
            self.parent_cell = parent_cell
            self.node_status = node_status

        def set_parent(self, parent_cell: Cell):
            self.parent_cell = parent_cell

        def set_status(self, status: DFSNodeStatus):
            self.node_status = status

    def __init__(self, map_: Map, starting_cell: Cell):
        self.map = map_

        # state
        self.open_list = []
        self.visited = set()
        self.next_cell = starting_cell

        self.n_rows = 16
        self.n_cols = 16

        # create a structure similar to the map for faster access
        self.known_cells = []
        for row in range(self.map.n_rows):
            row_arr = []
            for col in range(self.map.n_cols):
                row_arr.append(DepthFirstExploration.Node(self.map[row][col]))
            self.known_cells.append(row_arr)

        self.root_node = self.known_cells[starting_cell.row][starting_cell.col]

    def step(self) -> Optional[Cell]:
        """
        Get the next cell to.

        This variation skips the starting node.
        """
        current_cell = self.next_cell
        if current_cell not in self.visited:
            self.visited.add(current_cell)
            self.known_cells[current_cell.row][current_cell.col].set_status(DFSNodeStatus.VISITED)
            # expand node
            neighbors = self.map.get_neighbours(current_cell)
            for neighbor in neighbors:
                if neighbor not in self.visited:
                    self.open_list.append(neighbor)
                    self.known_cells[neighbor.row][neighbor.col].set_status(DFSNodeStatus.OPEN)
                    self.known_cells[neighbor.row][neighbor.col].set_parent(current_cell)

        if not self.open_list:
            # finished
            return None

        # explore node
        self.next_cell = self.open_list.pop()

        return self.next_cell
