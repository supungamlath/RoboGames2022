import sys
sys.path.append('../dave_controller')

from modules.anglr import Angle
import math
from maze import Maze

TWO_PI = Angle(0)
PI = Angle(math.pi)
HALF_PI = Angle(math.pi / 2)
QUARTER_PI = Angle(math.pi / 4)


# def test_anglr():
    # print(Angle(pi / 6) + Angle(2 * pi / 3))
    # print(x * 2 + Angle(3 * pi / 4) / 4 + 5 * Angle(pi / 3))
    # print(-abs(+Angle(pi)))
    # print(round(Angle(-75.87)))
    # print(Angle(-4.3) <= Angle(pi / 4) > Angle(0.118) == Angle(0.118))
    # print(Angle(-870.3, "gradians").normalized())
    # print(Angle(-870.3, "gradians").normalized(0)) # same as above
    # print(Angle(-870.3, "gradians").normalized(0, 2 * pi)) # same as above
    # print(Angle(-870.3, "gradians").normalized(-pi, pi))
    # print(Angle(-870.3, "gradians").normalized(-pi, 0))
    # print(Angle(1, "degrees").angle_between_clockwise(Angle(0, "degrees")))
    # print(Angle(1, "degrees").angle_between(Angle(0, "degrees")))
    # print(Angle(0, "degrees").angle_within(Angle(-45, "degrees"), Angle(45, "degrees")))
    # print(Angle(-1, "degrees").angle_within(Angle(-1, "degrees"), Angle(1, "degrees"), strictly_within=True))
    # print(Angle(-1, "degrees").angle_to(Angle(180, "degrees")))
    # print(Angle(0, "degrees").angle_to(Angle(180, "degrees")))


def test_anglr_within():
    angle = Angle(10, "degrees")
    assert angle.angleWithin(QUARTER_PI, 0) == True
    angle = Angle(-0.8)
    assert angle == TWO_PI - Angle(0.8)
    assert angle.angleWithin(0, TWO_PI - QUARTER_PI) == True

def test_anglr_between_clockwise():
    start_angle = Angle(0)
    target_angle = QUARTER_PI
    assert start_angle.angleBetweenCW(target_angle) == QUARTER_PI

    start_angle = Angle(360, "degrees")
    target_angle = QUARTER_PI
    assert start_angle.angleBetweenCW(target_angle) == QUARTER_PI

    start_angle = Angle(10, "degrees")
    target_angle = HALF_PI
    assert start_angle.angleBetweenCW(target_angle) == Angle(80, "degrees")

    start_angle = Angle(10, "degrees")
    target_angle = TWO_PI
    assert start_angle.angleBetweenCW(target_angle) == Angle(350, "degrees")

    start_angle = Angle(315, "degrees")
    target_angle = TWO_PI
    assert start_angle.angleBetweenCW(target_angle) == QUARTER_PI

    start_angle = Angle(315, "degrees")
    target_angle = QUARTER_PI
    assert start_angle.angleBetweenCW(target_angle) == HALF_PI

def test_anglr_between_counter_clockwise():
    start_angle = Angle(0)
    target_angle = QUARTER_PI
    assert start_angle.angleBetweenCCW(target_angle) == Angle(360 - 45, "degrees")

    start_angle = Angle(45, "degrees")
    target_angle = Angle(-45, "degrees")
    assert start_angle.angleBetweenCCW(target_angle) == HALF_PI

    start_angle = TWO_PI
    target_angle = Angle(10, "degrees")
    assert start_angle.angleBetweenCCW(target_angle) == Angle(350, "degrees")

    start_angle = Angle(180, "degrees")
    target_angle = Angle(135, "degrees")
    assert start_angle.angleBetweenCCW(target_angle) == QUARTER_PI

    start_angle = Angle(270, "degrees")
    target_angle = PI
    assert start_angle.angleBetweenCCW(target_angle) == HALF_PI

def test_direction_turns():
    maze = Maze()
    direction = "right"
    start_orientation = "E"
    if direction == "right":
        expected_bearing = Angle(maze.directionToAngle[start_orientation]) + PI / 2

    elif direction == "left":
        expected_bearing = Angle(maze.directionToAngle[start_orientation]) - PI / 2
    assert expected_bearing == Angle(180, "degrees")

if __name__ == '__main__':
    test_anglr_within()
    test_anglr_between_clockwise()
    test_anglr_between_counter_clockwise()
    test_direction_turns()
