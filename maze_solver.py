import random, datetime, csv, os
from collections import deque


class maze:
    """
    This is the main class to create maze.
    """

    def __init__(self, rows=10, cols=10):
        """
        rows--> No. of rows of the maze
        cols--> No. of columns of the maze
        Need to pass just the two arguments. The rest will be assigned automatically
        maze_map--> Will be set to a Dicationary. Keys will be cells and
                    values will be another dictionary with keys=['E','W','N','S'] for
                    East West North South and values will be 0 or 1. 0 means that
                    direction(EWNS) is blocked. 1 means that direction is open.
        grid--> A list of all cells
        path--> Shortest path from start(bottom right) to goal(by default top left)
                It will be a dictionary
        _win,_cell_width,_canvas -->    _win and )canvas are for Tkinter window and canvas
                                        _cell_width is cell width calculated automatically
        _agents-->  A list of aganets on the maze
        markedCells-->  Will be used to mark some particular cell during
                        path trace by the agent.
        _
        """
        self.rows = rows
        self.cols = cols
        self.maze_map = {}
        self.grid = []
        self.path = {}

    def _Open_East(self, x, y):
        """
        To remove the East Wall of the cell
        """
        self.maze_map[x, y]["E"] = 1
        if y + 1 <= self.cols:
            self.maze_map[x, y + 1]["W"] = 1

    def _Open_West(self, x, y):
        self.maze_map[x, y]["W"] = 1
        if y - 1 > 0:
            self.maze_map[x, y - 1]["E"] = 1

    def _Open_North(self, x, y):
        self.maze_map[x, y]["N"] = 1
        if x - 1 > 0:
            self.maze_map[x - 1, y]["S"] = 1

    def _Open_South(self, x, y):
        self.maze_map[x, y]["S"] = 1
        if x + 1 <= self.rows:
            self.maze_map[x + 1, y]["N"] = 1

    def CreateMaze(
        self,
        x=1,
        y=1,
        pattern=None,
        loopPercent=100,
        saveMaze=True,
    ):
        """
        One very important function to create a Random Maze
        pattern-->  It can be 'v' for vertical or 'h' for horizontal
                    Just the visual look of the maze will be more vertical/horizontal
                    passages will be there.
        loopPercent-->  0 means there will be just one path from start to goal (perfect maze)
                        Higher value means there will be multiple paths (loops)
                        Higher the value (max 100) more will be the loops
        saveMaze--> To save the generated Maze as CSV file for future reference.
        loadMaze--> Provide the CSV file to generate a desried maze
        """
        _stack = []
        _closed = []
        self._goal = (x, y)

        def blockedNeighbours(cell):
            n = []
            for d in self.maze_map[cell].keys():
                if self.maze_map[cell][d] == 0:
                    if d == "E" and (cell[0], cell[1] + 1) in self.grid:
                        n.append((cell[0], cell[1] + 1))
                    elif d == "W" and (cell[0], cell[1] - 1) in self.grid:
                        n.append((cell[0], cell[1] - 1))
                    elif d == "N" and (cell[0] - 1, cell[1]) in self.grid:
                        n.append((cell[0] - 1, cell[1]))
                    elif d == "S" and (cell[0] + 1, cell[1]) in self.grid:
                        n.append((cell[0] + 1, cell[1]))
            return n

        def removeWallinBetween(cell1, cell2):
            """
            To remove wall in between two cells
            """
            if cell1[0] == cell2[0]:
                if cell1[1] == cell2[1] + 1:
                    self.maze_map[cell1]["W"] = 1
                    self.maze_map[cell2]["E"] = 1
                else:
                    self.maze_map[cell1]["E"] = 1
                    self.maze_map[cell2]["W"] = 1
            else:
                if cell1[0] == cell2[0] + 1:
                    self.maze_map[cell1]["N"] = 1
                    self.maze_map[cell2]["S"] = 1
                else:
                    self.maze_map[cell1]["S"] = 1
                    self.maze_map[cell2]["N"] = 1

        def isCyclic(cell1, cell2):
            """
            To avoid too much blank(clear) path.
            """
            ans = False
            if cell1[0] == cell2[0]:
                if cell1[1] > cell2[1]:
                    cell1, cell2 = cell2, cell1
                if self.maze_map[cell1]["S"] == 1 and self.maze_map[cell2]["S"] == 1:
                    if (cell1[0] + 1, cell1[1]) in self.grid and self.maze_map[
                        (cell1[0] + 1, cell1[1])
                    ]["E"] == 1:
                        ans = True
                if self.maze_map[cell1]["N"] == 1 and self.maze_map[cell2]["N"] == 1:
                    if (cell1[0] - 1, cell1[1]) in self.grid and self.maze_map[
                        (cell1[0] - 1, cell1[1])
                    ]["E"] == 1:
                        ans = True
            else:
                if cell1[0] > cell2[0]:
                    cell1, cell2 = cell2, cell1
                if self.maze_map[cell1]["E"] == 1 and self.maze_map[cell2]["E"] == 1:
                    if (cell1[0], cell1[1] + 1) in self.grid and self.maze_map[
                        (cell1[0], cell1[1] + 1)
                    ]["S"] == 1:
                        ans = True
                if self.maze_map[cell1]["W"] == 1 and self.maze_map[cell2]["W"] == 1:
                    if (cell1[0], cell1[1] - 1) in self.grid and self.maze_map[
                        (cell1[0], cell1[1] - 1)
                    ]["S"] == 1:
                        ans = True
            return ans

        def BFS(cell):
            """
            Breadth First Search
            To generate the shortest path.
            This will be used only when there are multiple paths (loopPercent>0) or
            Maze is loaded from a CSV file.
            If a perfect maze is generated and without the load file, this method will
            not be used since the Maze generation will calculate the path.
            """
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
            while cell != (self.rows, self.cols):
                try:
                    fwdPath[path[cell]] = cell
                    cell = path[cell]
                except:
                    print("Path to goal not found!")
                    return
            return fwdPath

        # if maze is to be generated randomly
        _stack.append((x, y))
        _closed.append((x, y))
        biasLength = 2  # if pattern is 'v' or 'h'
        if pattern is not None and pattern.lower() == "h":
            biasLength = max(self.cols // 10, 2)
        if pattern is not None and pattern.lower() == "v":
            biasLength = max(self.rows // 10, 2)
        bias = 0

        while len(_stack) > 0:
            cell = []
            bias += 1
            if (x, y + 1) not in _closed and (x, y + 1) in self.grid:
                cell.append("E")
            if (x, y - 1) not in _closed and (x, y - 1) in self.grid:
                cell.append("W")
            if (x + 1, y) not in _closed and (x + 1, y) in self.grid:
                cell.append("S")
            if (x - 1, y) not in _closed and (x - 1, y) in self.grid:
                cell.append("N")
            if len(cell) > 0:
                if (
                    pattern is not None
                    and pattern.lower() == "h"
                    and bias <= biasLength
                ):
                    if "E" in cell or "W" in cell:
                        if "S" in cell:
                            cell.remove("S")
                        if "N" in cell:
                            cell.remove("N")
                elif (
                    pattern is not None
                    and pattern.lower() == "v"
                    and bias <= biasLength
                ):
                    if "N" in cell or "S" in cell:
                        if "E" in cell:
                            cell.remove("E")
                        if "W" in cell:
                            cell.remove("W")
                else:
                    bias = 0
                current_cell = random.choice(cell)
                if current_cell == "E":
                    self._Open_East(x, y)
                    self.path[x, y + 1] = x, y
                    y = y + 1
                    _closed.append((x, y))
                    _stack.append((x, y))

                elif current_cell == "W":
                    self._Open_West(x, y)
                    self.path[x, y - 1] = x, y
                    y = y - 1
                    _closed.append((x, y))
                    _stack.append((x, y))

                elif current_cell == "N":
                    self._Open_North(x, y)
                    self.path[(x - 1, y)] = x, y
                    x = x - 1
                    _closed.append((x, y))
                    _stack.append((x, y))

                elif current_cell == "S":
                    self._Open_South(x, y)
                    self.path[(x + 1, y)] = x, y
                    x = x + 1
                    _closed.append((x, y))
                    _stack.append((x, y))

            else:
                x, y = _stack.pop()

        ## Multiple Path Loops
        if loopPercent != 0:

            x, y = self.rows, self.cols
            pathCells = [(x, y)]
            while x != self.rows or y != self.cols:
                x, y = self.path[(x, y)]
                pathCells.append((x, y))
            notPathCells = [i for i in self.grid if i not in pathCells]
            random.shuffle(pathCells)
            random.shuffle(notPathCells)
            pathLength = len(pathCells)
            notPathLength = len(notPathCells)
            count1, count2 = (
                pathLength / 3 * loopPercent / 100,
                notPathLength / 3 * loopPercent / 100,
            )

            # remove blocks from shortest path cells
            count = 0
            i = 0
            while count < count1:  # these many blocks to remove
                if len(blockedNeighbours(pathCells[i])) > 0:
                    cell = random.choice(blockedNeighbours(pathCells[i]))
                    if not isCyclic(cell, pathCells[i]):
                        removeWallinBetween(cell, pathCells[i])
                        count += 1
                    i += 1

                else:
                    i += 1
                if i == len(pathCells):
                    break
            # remove blocks from outside shortest path cells
            if len(notPathCells) > 0:
                count = 0
                i = 0
                while count < count2:  # these many blocks to remove
                    if len(blockedNeighbours(notPathCells[i])) > 0:
                        cell = random.choice(blockedNeighbours(notPathCells[i]))
                        if not isCyclic(cell, notPathCells[i]):
                            removeWallinBetween(cell, notPathCells[i])
                            count += 1
                        i += 1

                    else:
                        i += 1
                    if i == len(notPathCells):
                        break
            self.path = BFS((self.rows, self.cols))


        if saveMaze:
            dt_string = datetime.datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
            with open(f"maze--{dt_string}.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["  cell  ", "E", "W", "N", "S"])
                for k, v in self.maze_map.items():
                    entry = [k]
                    for i in v.values():
                        entry.append(i)
                    writer.writerow(entry)
                f.seek(0, os.SEEK_END)
                f.seek(f.tell() - 2, os.SEEK_SET)
                f.truncate()

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


