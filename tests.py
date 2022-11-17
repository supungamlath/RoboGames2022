from pyamaze import *

m = maze(9, 8)
m.CreateMaze(loadMaze="saved_maze.csv")
a = agent(m, x=6, y=4, footprints=True)
# m.tracePath({a:m.path})
# print(m.path)
m.run()
