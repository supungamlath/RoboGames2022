import math
from modules.anglr import Angle
from maze import Maze

### Math Constants ###
TWO_PI = Angle(math.pi * 2)
PI = Angle(math.pi)
HALF_PI = Angle(math.pi / 2)
QUARTER_PI = Angle(math.pi / 4)

class Position:
    def __init__(self, x, y, heading):
        # x axis is the vertical axis in Webots
        self.x = x
        # y axis is the horizontal axis in Webots
        self.y = y
        # Positive direction of Webots x axis is 0 radians
        self.heading = heading

    def getWorldXY(self):
        return self.x, self.y

    def getMazeCell(self):
        return Maze.worldCoordToMazeCell((self.x, self.y))

    def getHeading(self):
        return self.heading

    def getOrientation(self):
        if self.heading < QUARTER_PI or self.heading >= (TWO_PI - QUARTER_PI):
            return "N"
        elif QUARTER_PI <= self.heading < (HALF_PI + QUARTER_PI):
            return "E"
        elif (HALF_PI + QUARTER_PI) <= self.heading < (PI + QUARTER_PI):
            return "S"
        elif (PI + QUARTER_PI) <= self.heading < (TWO_PI - QUARTER_PI):
            return "W"

    def getRelativeXY(self, position):
        orientation = self.getOrientation()
        if orientation == "N" or orientation == "S":
            return position[0], position[1]
        elif orientation == "E" or orientation == "W":
            return position[1], position[0]

    def getRelativeVAxisError(self, target):
        orientation = self.getOrientation()
        if orientation == "N":
            return self.y - target
        elif orientation == "S":
            return target - self.y
        elif orientation == "E":
            return self.x - target
        elif orientation == "W":
            return target - self.x

    def isTargetReachableMovingFoward(self, target):
        orientation = self.getOrientation()
        if orientation == "N":
            return self.x < target
        elif orientation == "S":
            return self.x > target
        elif orientation == "E":
            return self.y > target
        elif orientation == "W":
            return self.y < target

    def getDistanceTo(self, position):
        if isinstance(position, Position):
            t_x, t_y = position.getWorldXY()
        else:
            t_x = position[0]
            t_y = position[1]
        dx = t_x - self.x
        dy = t_y - self.y
        return math.sqrt(dx**2 + dy**2)

    def getClosestPosition(self, positions):
        return min(positions, key=lambda pos:self.getDistanceTo(pos))

    def getFarthestPosition(self, positions):
        return max(positions, key=lambda pos:self.getDistanceTo(pos))

    def getPositionsInRadius(self, positions, radius):
        return [position for position in positions if self.getDistanceTo(position) <= radius]

    def isPositionInRadius(self, position, radius):
        return self.getDistanceTo(position) <= radius

    def getPositionsSortedByDistance(self, positions):
        return sorted(positions, key=lambda pos:self.getDistanceTo(pos))
