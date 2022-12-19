import csv, shutil, cv2, math, numpy as np
from heapq import heappush, heappop
from time import time
from collections import deque
from tempfile import NamedTemporaryFile

CLOSE_TO_WALL_THRESHOLD = 2
FILE_WRITE_PERIOD = 1

class Maze:
    def __init__(self, rows, cols, filename="saved_maze.csv"):
        self.rows = rows
        self.cols = cols
        self.maze_map = {}
        self.grid = []
        self.path = {}
        self.filename = filename
        self.fields = ["  cell  ", "state"]
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
                    if i == 1:
                        parts[1] = "0"
                    elif i == self.rows:
                        parts[1] = "0"
                    if j == 1:
                        parts[1] = "0"
                    elif j == self.cols:
                        parts[1] = "0"
                    line = ",".join(parts) + "\n" 
                    lines.append(line)
            f.writelines(lines)

    def loadMaze(self):
        # Load maze from CSV file
        with open(self.filename, "r", newline="") as f:
            for row in csv.DictReader(f):
                cell = [int(i) for i in row["  cell  "].strip("()").split(", ")]
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
        with open(self.filename, "r", newline="") as csvfile, tempfile:
            reader = csv.DictReader(csvfile, fieldnames=self.fields)
            writer = csv.DictWriter(tempfile, fieldnames=self.fields)

            for row in reader:
                row["state"] = self.maze_map[row["  cell  "]]
                writer.writerow(row)

        shutil.move(tempfile.name, self.filename)

    def addDataToMaze(self, cell, state):
        self.maze_map[cell] = state
        if time() - self.last_save_time > FILE_WRITE_PERIOD:
            self.saveMaze()
            self.last_save_time = time()

    def addDataIfUnknown(self, cell, state):
        if self.maze_map[cell] == -1:
            self.addDataToMaze(cell, state)

    def isCloseToWall(self, cell):
        for dy in range(-CLOSE_TO_WALL_THRESHOLD, CLOSE_TO_WALL_THRESHOLD + 1):
            for dx in range(-CLOSE_TO_WALL_THRESHOLD, CLOSE_TO_WALL_THRESHOLD + 1):
                if self.maze_map[(cell[0] + dy, cell[1] + dx)] == 0:
                    return True
        return False

    # Calculate the Manhattan distance between two cells
    def getManhattanDistance(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
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
                    priority = new_cost + self.getManhattanDistance((next_row, next_col), goal)
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

    def showMaze(self):
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

        # Show the image with the rectangles
        cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
        
        # Using resizeWindow()
        cv2.resizeWindow("Resized_Window", self.rows*3, self.cols*3)
        
        # Displaying the image
        cv2.imshow("Resized_Window", image)

        # Wait for a key press to close the window
        cv2.waitKey(0)

        # Destroy all windows
        cv2.destroyAllWindows()


