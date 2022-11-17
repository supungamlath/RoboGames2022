import csv
from collections import deque


class maze:

    def __init__(self, rows, cols):

        self.rows = rows
        self.cols = cols
        self.maze_map = {}
        self.grid = []
        self.path = {}

    def solveMaze(
        self,
        start=(1, 1),
        goal=(1, 1),
        path=None,
    ):

        self._start = start
        self._goal = goal

        def BFS(cell):
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
            fwdPath = {}
            cell = self._goal
            while cell != self._start:
                try:
                    fwdPath[path[cell]] = cell
                    cell = path[cell]
                except:
                    print("Path to goal not found!")
                    return
            return fwdPath

        # Load maze from CSV file
        with open(path, "r") as f:
            last = list(f.readlines())[-1]
            c = last.split(",")
            c[0] = int(c[0].lstrip('"('))
            c[1] = int(c[1].rstrip(')"'))
            self.rows = c[0]
            self.cols = c[1]
            self.grid = []

        with open(path, "r") as f:
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
        self.path = BFS(start)


