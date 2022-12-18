import csv, shutil
from collections import deque
from tempfile import NamedTemporaryFile

class Maze:
    def __init__(self, rows, cols, filename="saved_maze.csv"):
        self.rows = rows
        self.cols = cols
        self.maze_map = {}
        self.grid = []
        self.path = {}
        self.filename = filename
        self.fields = ["  cell  ", "E", "W", "N", "S"]
   
        with open(self.filename, "w") as f:
            lines = [",".join(self.fields) + "\n"]
            for i in range(1, self.rows + 1):
                for j in range(1, self.cols + 1):
                    parts = ["\"" + str((i, j)) + "\""]
                    parts.extend(["-1"] * 4)
                    if i == 1:
                        parts[3] = "0"
                    elif i == self.rows:
                        parts[4] = "0"
                    if j == 1:
                        parts[2] = "0"
                    elif j == self.cols:
                        parts[1] = "0"
                    line = ",".join(parts) + "\n" 
                    lines.append(line)
            f.writelines(lines)

    def BFS(self, start, goal):
        cell = start
        frontier = deque()
        frontier.append(cell)
        path = {}
        visited = {(self.rows, self.cols)}
        while len(frontier) > 0:
            cell = frontier.popleft()
            if self.maze_map[cell]["W"] and (cell[0], cell[1] - 1) not in visited:
                nextCell = (cell[0], cell[1] - 1)
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.maze_map[cell]["S"] and (cell[0] + 1, cell[1]) not in visited:
                nextCell = (cell[0] + 1, cell[1])
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.maze_map[cell]["E"] and (cell[0], cell[1] + 1) not in visited:
                nextCell = (cell[0], cell[1] + 1)
                path[nextCell] = cell
                frontier.append(nextCell)
                visited.add(nextCell)
            if self.maze_map[cell]["N"] and (cell[0] - 1, cell[1]) not in visited:
                nextCell = (cell[0] - 1, cell[1])
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

    def loadMaze(self):
        # Load maze from CSV file
        with open(self.filename, "r", newline="") as f:
            for row in csv.DictReader(f):
                cell = [int(i) for i in row["  cell  "].strip("()").split(", ")]
                self.maze_map[tuple(cell)] = {
                    "E": int(row["E"]),
                    "W": int(row["W"]),
                    "N": int(row["N"]),
                    "S": int(row["S"])}

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
    def getAdjacentCell(cell, direction):
        if direction == "N":
            return (cell[0] - 1, cell[1])
        elif direction == "E":
            return (cell[0], cell[1] + 1)
        elif direction == "S":
            return (cell[0] + 1, cell[1])
        elif direction == "W":
            return (cell[0], cell[1] - 1)

    def addDataToMaze(self, cell, direction, wall_present):
        tempfile = NamedTemporaryFile("w+t", newline="", delete=False)
        with open(self.filename, "r", newline="") as csvfile, tempfile:
            reader = csv.DictReader(csvfile, fieldnames=self.fields)
            writer = csv.DictWriter(tempfile, fieldnames=self.fields)

            adj_cell = self.getAdjacentCell(cell, direction)
            for row in reader:
                if row["  cell  "] == str(cell):
                    row[direction] = wall_present
                elif row["  cell  "] == str(adj_cell):
                    row[self.getOppositeDir(direction)] = wall_present

                writer.writerow(row)

        shutil.move(tempfile.name, self.filename)
        self.maze_map[cell][direction] = wall_present

    def solveMaze(self, start, goal):
        self.path = self.BFS(start, goal)

    def getPath(self, start, goal):
        return self.BFS(start, goal)

    def loadAndSolveMaze(self, start, goal):
        self.loadMaze()
        self.solveMaze(start, goal)
