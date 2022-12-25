import sys, pytest
sys.path.append('../dave_controller')

from slam import SLAM
import math

def test_slam_m_line():
    slam = SLAM(None, None, None, None)
    point1 = (1, 2)
    point2 = (3, 4)
    m, c = slam.setMLine(point1, point2)
    print(math.atan(0))
    assert m == 1
    assert c == 1
    
def test_slam_angle_to_goal():
    slam = SLAM(None, None, None, None)

    point1 = (1, 1)
    point2 = (2, 1)
    angle = slam.getAngleToGoal(point1, point2)
    assert pytest.approx(angle, 0.000000001) == math.radians(0)
    
    point1 = (1, 2)
    point2 = (2, 1)
    angle = slam.getAngleToGoal(point1, point2)
    assert pytest.approx(angle, 0.000000001) == math.radians(45)

    point1 = (1, 3)
    point2 = (0, 2)
    angle = slam.getAngleToGoal(point1, point2)
    assert pytest.approx(angle, 0.000000001) == math.radians(90 + 45)

    point1 = (1, 3)
    point2 = (0, 4)
    angle = slam.getAngleToGoal(point1, point2)
    assert pytest.approx(angle, 0.000000001) == math.radians(180 + 45)

    point1 = (1, 1)
    point2 = (2, 2)
    angle = slam.getAngleToGoal(point1, point2)
    assert pytest.approx(angle, 0.000000001) == math.radians(360 - 45)

    point1 = (-2.09, -0.015)
    point2 = (-1.43, 0.1)
    angle = slam.getAngleToGoal(point1, point2)
    print(angle)


if __name__ == '__main__':
    test_slam_m_line()
    test_slam_angle_to_goal()