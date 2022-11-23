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
        self.empty_maze = """  cell  ,E,W,N,S
                        "(1, 1)",-1,0,0,-1
                        "(2, 1)",-1,0,-1,-1
                        "(3, 1)",-1,0,-1,-1
                        "(4, 1)",-1,0,-1,-1
                        "(5, 1)",-1,0,-1,-1
                        "(6, 1)",-1,0,-1,-1
                        "(7, 1)",-1,0,-1,-1
                        "(8, 1)",-1,0,-1,0
                        "(9, 1)",-1,0,0,0
                        "(1, 2)",-1,-1,0,-1
                        "(2, 2)",-1,-1,-1,-1
                        "(3, 2)",-1,-1,-1,-1
                        "(4, 2)",-1,-1,-1,-1
                        "(5, 2)",-1,-1,-1,-1
                        "(6, 2)",-1,-1,-1,-1
                        "(7, 2)",-1,-1,-1,-1
                        "(8, 2)",-1,-1,-1,0
                        "(9, 2)",-1,-1,0,0
                        "(1, 3)",-1,-1,0,-1
                        "(2, 3)",-1,-1,-1,-1
                        "(3, 3)",-1,-1,-1,-1
                        "(4, 3)",-1,-1,-1,-1
                        "(5, 3)",-1,-1,-1,-1
                        "(6, 3)",-1,-1,-1,-1
                        "(7, 3)",-1,-1,-1,-1
                        "(8, 3)",-1,-1,-1,0
                        "(9, 3)",0,-1,0,0
                        "(1, 4)",-1,-1,0,-1
                        "(2, 4)",-1,-1,-1,-1
                        "(3, 4)",-1,-1,-1,-1
                        "(4, 4)",-1,-1,-1,-1
                        "(5, 4)",-1,-1,-1,-1
                        "(6, 4)",-1,-1,-1,-1
                        "(7, 4)",-1,-1,-1,-1
                        "(8, 4)",-1,-1,-1,-1
                        "(9, 4)",-1,0,-1,0
                        "(1, 5)",-1,-1,0,-1
                        "(2, 5)",-1,-1,-1,-1
                        "(3, 5)",-1,-1,-1,-1
                        "(4, 5)",-1,-1,-1,-1
                        "(5, 5)",-1,-1,-1,-1
                        "(6, 5)",-1,-1,-1,-1
                        "(7, 5)",-1,-1,-1,-1
                        "(8, 5)",-1,-1,-1,-1
                        "(9, 5)",0,-1,-1,0
                        "(1, 6)",-1,-1,0,-1
                        "(2, 6)",-1,-1,-1,-1
                        "(3, 6)",-1,-1,-1,-1
                        "(4, 6)",-1,-1,-1,-1
                        "(5, 6)",-1,-1,-1,-1
                        "(6, 6)",-1,-1,-1,-1
                        "(7, 6)",-1,-1,-1,-1
                        "(8, 6)",-1,-1,-1,0
                        "(9, 6)",-1,0,0,0
                        "(1, 7)",-1,-1,0,-1
                        "(2, 7)",-1,-1,-1,-1
                        "(3, 7)",-1,-1,-1,-1
                        "(4, 7)",-1,-1,-1,-1
                        "(5, 7)",-1,-1,-1,-1
                        "(6, 7)",-1,-1,-1,-1
                        "(7, 7)",-1,-1,-1,-1
                        "(8, 7)",-1,-1,-1,0
                        "(9, 7)",-1,-1,0,0
                        "(1, 8)",0,-1,0,-1
                        "(2, 8)",0,-1,-1,-1
                        "(3, 8)",0,-1,-1,-1
                        "(4, 8)",0,-1,-1,-1
                        "(5, 8)",0,-1,-1,-1
                        "(6, 8)",0,-1,-1,-1
                        "(7, 8)",0,-1,-1,-1
                        "(8, 8)",0,-1,-1,0
                        "(9, 8)",0,-1,0,0
        """
        with open(self.filename, "w") as f:
            f.write(self.empty_maze)

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
            except:
                print("Path from", start, "to", goal, "does not exist")
                return
        return path_dict

    def loadMaze(self):
        # Load maze from CSV file
        with open(self.filename, "r") as f:
            last = list(f.readlines())[-1]
            c = last.split(",")
            c[0] = int(c[0].lstrip('"('))
            c[1] = int(c[1].rstrip(')"'))
            self.rows = c[0]
            self.cols = c[1]
            self.grid = []

        with open(self.filename, "r") as f:
            r = csv.reader(f)
            next(r)
            for i in r:
                c = i[0].split(",")
                c[0] = int(c[0].lstrip("("))
                c[1] = int(c[1].rstrip(")"))
                self.maze_map[tuple(c)] = {
                    "E": int(i[1]),
                    "W": int(i[2]),
                    "N": int(i[3]),
                    "S": int(i[4]),
                }

    def addWallToMaze(self, cell, direction, wall_present):
        def getOppositeDir(direction):
            if direction == "N":
                return "S"
            elif direction == "E":
                return "W"
            elif direction == "S":
                return "N"
            elif direction == "W":
                return "E"

        def getAdjacentCell(cell, direction):
            if direction == "N":
                return (cell[0] - 1, cell[1])
            elif direction == "E":
                return (cell[0], cell[1] + 1)
            elif direction == "S":
                return (cell[0] + 1, cell[1])
            elif direction == "W":
                return (cell[0], cell[1] - 1)

        fields = ["  cell  ", "E", "W", "N", "S"]
        tempfile = NamedTemporaryFile("w+t", newline="", delete=False)
        with open(self.filename, "r", newline="") as csvfile, tempfile:
            reader = csv.DictReader(csvfile, fieldnames=fields)
            writer = csv.DictWriter(tempfile, fieldnames=fields)

            adj_cell = getAdjacentCell(cell, direction)
            for row in reader:
                if row["  cell  "] == str(cell):
                    row[direction] = wall_present
                elif row["  cell  "] == str(adj_cell):
                    row[getOppositeDir(direction)] = wall_present

                writer.writerow(row)

        shutil.move(tempfile.name, self.filename)

    def solveMaze(self, start, goal):
        self.path = self.BFS(start, goal)

    def getPathLength(self, start, goal):
        return len(self.BFS(start, goal))

    def loadAndSolveMaze(self, start, goal):
        self.loadMaze()
        self.solveMaze(start, goal)
