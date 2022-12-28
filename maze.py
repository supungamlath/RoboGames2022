import csv, shutil, cv2, math, numpy as np
from heapq import heappush, heappop
from time import time
from collections import deque
from tempfile import NamedTemporaryFile
from image_processing import *

CLOSE_TO_WALL_THRESHOLD = 1
FILE_WRITE_PERIOD = 2

class Maze:
    def __init__(self, rows, cols, filename="saved_maze.csv"):
        self.rows = rows
        self.cols = cols
        self.maze_map = {}
        self.grid = []
        self.path = {}
        self.filename = filename
        self.fields = ["cell", "state"]
        self.directionToAngle = {"N": 0, "E": math.pi/2, "S": math.pi, "W": 3*math.pi/2}
        self.last_save_time = time()
    
    def resetMaze(self):
        self.createMazeFile()
        self.loadMaze()
   
    def createMazeFile(self):
        self.last_save_time = time()
        with open(self.filename, "w") as f:
            lines = [",".join(self.fields) + "\n"]
            for i in range(1, self.rows + 1):
                for j in range(1, self.cols + 1):
                    parts = ["\"" + str((i, j)) + "\""]
                    parts.append("-1")
                    if i == 1 or i == self.rows or j == 1 or j == self.cols:
                        parts[1] = "0"
                    line = ",".join(parts) + "\n" 
                    lines.append(line)
            f.writelines(lines)

    def loadMaze(self):
        # Load maze from CSV file
        with open(self.filename, "r", newline="") as f:
            for row in csv.DictReader(f):
                cell = [int(i) for i in row["cell"].strip("()").split(", ")]
                self.maze_map[tuple(cell)] = int(row["state"])

    @staticmethod
    def getOppositeDir(direction):
        if direction == "N":
            return "S"
        elif direction == "E":
            return "W"
        elif direction == "S":
            return "N"
        elif direction == "W":
            return "E"

    @staticmethod
    def getLeftDir(direction):
        if direction == "N":
            return "W"
        elif direction == "E":
            return "N"
        elif direction == "S":
            return "E"
        elif direction == "W":
            return "S"

    @staticmethod
    def getRightDir(direction):
        left_dir = Maze.getLeftDir(direction)
        return Maze.getOppositeDir(left_dir)

    @staticmethod
    def getCellInDirection(cell, direction, offset):
        if direction == "N":
            return (cell[0] - offset, cell[1])
        elif direction == "E":
            return (cell[0], cell[1] + offset)
        elif direction == "S":
            return (cell[0] + offset, cell[1])
        elif direction == "W":
            return (cell[0], cell[1] - offset)

    """
    Returns the cell in the given direction and offset from the given cell. 
    The offset is a dictionary with the keys "parallel-axis" and "cross-axis".
    "parallel-axis" is the offset in the positive (infront) direction of the given direction, 
    and "cross-axis" is the offset in the perpendicular direction (left) to the given direction.
    """
    @staticmethod
    def getCellInOffsetDirection(cell, direction, offset={"parallel-axis":0, "cross-axis":0}):
        if direction == "N":
            return (cell[0] - offset["parallel-axis"], cell[1] - offset["cross-axis"])
        elif direction == "E":
            return (cell[0] - offset["cross-axis"], cell[1] + offset["parallel-axis"])
        elif direction == "S":
            return (cell[0] + offset["parallel-axis"], cell[1] + offset["cross-axis"])
        elif direction == "W":
            return (cell[0] + offset["cross-axis"], cell[1] - offset["parallel-axis"])

    @staticmethod
    def getAdjacentCell(cell, direction):
        return Maze.getCellInDirection(cell, direction, 1)

    def getAdjacentCellState(self, cell, direction):
        adj_cell = self.getAdjacentCell(cell, direction)
        if adj_cell in self.maze_map:
            return self.maze_map[adj_cell]
        return 0

    def BFS(self, start, goal):
        cell = start
        frontier = deque()
        frontier.append(cell)
        path = {}
        visited = {start}
        while len(frontier) > 0:
            cell = frontier.popleft()
            if self.getAdjacentCellState(cell, "W") != 0 and self.getAdjacentCell(cell, "W") not in visited:
                nextCell = self.getAdjacentCell(cell, "W")
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.getAdjacentCellState(cell, "S") != 0 and self.getAdjacentCell(cell, "S") not in visited:
                nextCell = self.getAdjacentCell(cell, "S")
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.getAdjacentCellState(cell, "E") != 0 and self.getAdjacentCell(cell, "E") not in visited:
                nextCell = self.getAdjacentCell(cell, "E")
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.getAdjacentCellState(cell, "N") != 0 and self.getAdjacentCell(cell, "N") not in visited:
                nextCell = self.getAdjacentCell(cell, "N")
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
        path_dict = {}
        cell = goal
        while cell != start:
            try:
                path_dict[path[cell]] = cell
                cell = path[cell]
            except KeyError:
                print("Path from", start, "to", goal, "does not exist")
                return
        return path_dict

    def saveMaze(self):
        tempfile = NamedTemporaryFile("w+t", newline="", delete=False)
        with open(tempfile.name, "w", newline="") as tempfile:
            writer = csv.DictWriter(tempfile, fieldnames=self.fields)
            writer.writeheader()
            for cell, state in self.maze_map.items():
                row = {"cell":cell, "state":state}
                writer.writerow(row)

        shutil.move(tempfile.name, self.filename)

    def addDataToMaze(self, cell, state, fill_size = 0):
        for p in range(-fill_size, fill_size + 1):
            for c in range(-fill_size, fill_size + 1):
                self.maze_map[(cell[0] + p, cell[1] + c)] = state

        if time() - self.last_save_time > FILE_WRITE_PERIOD:
            self.saveMaze()
            self.last_save_time = time()

    def addDataIfUnknown(self, cell, state, fill_size = 0):
        if self.maze_map[cell] == -1:
            self.addDataToMaze(cell, state, fill_size)

    def isCellKnown(self, cell):
        return self.maze_map[cell] != -1

    def isCloseToWall(self, cell, threshold = CLOSE_TO_WALL_THRESHOLD):
        for dy in range(-threshold, threshold + 1):
            for dx in range(-threshold, threshold + 1):
                if self.maze_map[(cell[0] + dy, cell[1] + dx)] == 0:
                    return True
        return False

    # Calculate the Manhattan distance between two cells
    def getManhattanDistance(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def getHeuristic(self, cell, goal):
        manhattan_d = self.getManhattanDistance(cell, goal)
        if self.isCloseToWall(cell, threshold = CLOSE_TO_WALL_THRESHOLD + 1):
            return manhattan_d + 100
        return manhattan_d 

    # The A* search algorithm
    def AStarSearch(self, start, goal):
        # The cost of moving from one cell to an adjacent cell is 1
        cost = 1

        # Set of visited cells
        visited = set()
        # Priority queue of unexplored cells, ordered by their estimated cost
        heap = []
        # Map of cell coordinates to their parent cells (for reconstructing the path)
        came_from = {}
        # Map of cell coordinates to their cost (g-value)
        cost_so_far = {}
        
        # Add the start position to the heap with an estimated cost of 0
        heappush(heap, (0, start))
        cost_so_far[start] = 0
        
        # Keep searching until the heap is empty or the goal is found
        while heap:
            # Pop the cell with the lowest estimated cost
            _, current = heappop(heap)
            # If the current cell is the goal, return the path
            if current == goal:
                break
            # Mark the current cell as visited
            visited.add(current)
            # Get the row and column of the current cell
            row, col = current
            # Explore the four adjacent cells (up, down, left, right)
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                # Calculate the coordinates of the adjacent cell
                next_row, next_col = row + dy, col + dx
                # Skip the adjacent cell if it is out of bounds or is a wall
                if not (0 < next_row < self.rows and 0 < next_col < self.cols) or self.maze_map[(next_row, next_col)] == 0:
                    continue
                if self.isCloseToWall((next_row, next_col)):
                    continue
                # Skip the adjacent cell if it has already been visited
                if (next_row, next_col) in visited:
                    continue
                # Calculate the cost of moving to the adjacent cell
                new_cost = cost_so_far[current] + cost
                # If the adjacent cell has not been visited or the new cost is lower than the old cost,
                # update the cost and add the cell to the heap
                if (next_row, next_col) not in cost_so_far or new_cost < cost_so_far[(next_row, next_col)]:
                    cost_so_far[(next_row, next_col)] = new_cost
                    priority = new_cost + self.getHeuristic((next_row, next_col), goal)
                    heappush(heap, (priority, (next_row, next_col)))
                    came_from[(next_row, next_col)] = current
        # If the goal was not found, return an empty path
        else:
            print("Path from", start, "to", goal, "does not exist")
            return None

        # Reconstruct the path from start to goal
        path = {}
        current = goal
        while current != start:
            path[came_from[current]] = current
            current = came_from[current]

        return path

    def getPath(self, start, goal):
        # return self.BFS(start, goal)
        return self.AStarSearch(start, goal)

    def showMaze(self, path, current_cell, refresh_rate = 200):
        # Create an empty NumPy array
        image = np.zeros((self.rows, self.cols,3))

        # Iterate through the dictionary and set the corresponding pixels in the image
        for (row, col), value in self.maze_map.items():
            if value == 0:
                image[row-1][col-1] = (0,0,0)
            elif value == 1:
                image[row-1][col-1] = (0,255,0)
            else:
                image[row-1][col-1] = (255,255,255)

        if path:
            for (row, col) in path:
                image[row-1][col-1] = (0,0,255)

        if current_cell:
            image[current_cell[0]-1][current_cell[1]-1] = (255,0,0)

        # Show the image with the rectangles
        cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
        
        # Using resizeWindow()
        cv2.resizeWindow("Resized_Window", self.rows*3, self.cols*3)
        
        # Displaying the image
        cv2.imshow("Resized_Window", image)

        # Wait for a key press to close the window
        cv2.waitKey(refresh_rate)

        # Destroy all windows
        # cv2.destroyAllWindows()




