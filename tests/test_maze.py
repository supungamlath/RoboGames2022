import sys
sys.path.append('../dave_controller')
from maze import Maze

def test_show_maze():
    m = Maze()
    m.loadMaze()
    m.showMaze(None, None, refresh_rate = 0)

def test_get_cell_in_offset_direction():
    m = Maze()
    cell = (1, 1)
    offset_cell = m.getCellInOffsetDirection(cell, "N", {"parallel-axis":0, "cross-axis":0})
    assert offset_cell == (1, 1)

    cell = (6, 3)
    offset_cell = m.getCellInOffsetDirection(cell, "N", {"parallel-axis":1, "cross-axis":-1})
    assert offset_cell == (5, 4)

    cell = (6, 3)
    offset_cell = m.getCellInOffsetDirection(cell, "N", {"parallel-axis":1, "cross-axis":1})
    assert offset_cell == (5, 2)

    cell = (6, 3)
    offset_cell = m.getCellInOffsetDirection(cell, "W", {"parallel-axis":1, "cross-axis":2})
    assert offset_cell == (8, 2)

    cell = (6, 3)
    offset_cell = m.getCellInOffsetDirection(cell, "E", {"parallel-axis":1, "cross-axis":2})
    assert offset_cell == (4, 4)

    cell = (6, 3)
    offset_cell = m.getCellInOffsetDirection(cell, "S", {"parallel-axis":1, "cross-axis":2})
    assert offset_cell == (7, 5)

if __name__ == "__main__":
    test_show_maze()
    # test_get_cell_in_offset_direction()