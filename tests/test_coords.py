import sys, pytest
sys.path.append('../dave_controller')

GRID_SIZE = 222
SQUARE_LENGTH = 0.02

def worldCoordToMazeCell(world_coord):
    # Note that the world and maze have same x and y axes but different to usual Cartesian system
    # world_coord[0] is x, world_coord[1] is y
    # x is vertical and y is horizontal
    if GRID_SIZE % 2 == 0:
        x = (GRID_SIZE // 2) - int(world_coord[0] // SQUARE_LENGTH)
        y = (GRID_SIZE // 2) - int(world_coord[1] // SQUARE_LENGTH)
    else:
        x = (GRID_SIZE // 2 + 1) - int(world_coord[0] // (SQUARE_LENGTH / 2)) // 2
        y = (GRID_SIZE // 2 + 1) - int(world_coord[1] // (SQUARE_LENGTH / 2)) // 2
    # Maze cells are (vertical, horizontal)
    return (x, y)

def mazeCellToWorldCoords(maze_cell):
    # Note that the world and maze have x and y axes opposite to usual Cartesian system
    # maze_cell[0] is vertical, maze_cell[1] is horizontal
    if GRID_SIZE % 2 == 0 :
        x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
        y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
    else:
        x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH
        y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH
    return (y, x)

def test_world_to_maze():
    global GRID_SIZE, SQUARE_LENGTH
    GRID_SIZE = 222
    SQUARE_LENGTH = 0.02
    world_coord = (-1.91687, -0.0495742)
    maze_coord = worldCoordToMazeCell(world_coord)
    assert maze_coord == (207, 114)

    GRID_SIZE = 10
    SQUARE_LENGTH = 1
    world_coord = (-5, 5)
    maze_coord = worldCoordToMazeCell(world_coord)
    assert maze_coord == (10, 0)

    GRID_SIZE = 222
    SQUARE_LENGTH = 0.02
    world_coord = (0.25, 2.12)
    maze_coord = worldCoordToMazeCell(world_coord)
    print(maze_coord)

def test_maze_to_world():
    global GRID_SIZE, SQUARE_LENGTH
    GRID_SIZE = 222
    SQUARE_LENGTH = 0.02
    maze_cell = (207, 114)
    world_coord = mazeCellToWorldCoords(maze_cell)
    assert pytest.approx(world_coord, 0.00000001) == (-1.91, -0.05)
 
    GRID_SIZE = 222
    SQUARE_LENGTH = 0.02   
    maze_cell = (212, 112)
    world_coord = mazeCellToWorldCoords(maze_cell)
    print(world_coord)

if __name__ == "__main__":
    test_world_to_maze()
    test_maze_to_world()