# from maze_solver import *
from pyamaze import *

# m = maze(9, 8)
# # m.CreateMaze(loopPercent=100)
# m.solveMaze(start=(9, 4), goal=(1,2), path="saved_maze.csv")
# print(m.path)


m = maze(9, 8)
m.CreateMaze(loadMaze="saved_maze.csv")
a = agent(m, x=6, y=4, footprints=True)
# m.tracePath({a:m.path})
print(m.path)
m.run()
