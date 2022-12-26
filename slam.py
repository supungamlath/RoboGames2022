import logging, math

PROXIMITY_THRESHOLD = 100
CAMERA_TEST_PIXEL_ROW = 28
CELL_OFFSET = 1 # Wall data should be offset by this constant to account for the robot's diameter 
WANDERING_TIMEOUT = 5 # Seconds
MLINE_WIDTH = 0.01 # Meters
SAME_INTERSECTION_THRESHOLD = 0.07 # Meters
GOAL_REACHED_THRESHOLD = 0.1

# Class for Simultaneous Localization and Mapping
class SLAM:
    def __init__(self, maze, camera, distance_sensors, color_detector):
        self.maze = maze
        self.camera = camera
        self.distance_sensors = distance_sensors
        self.color_detector = color_detector
        self.maze_coord = None
        self.orientation = None
        self.m_line = None
        self.last_intersection_distance = float("inf")
        self.last_proximity_reading_time = 0

    # Function to detect whether object is in proximity to given sensor 
    def isObjectInProximity(self, sensor_name):
        logging.debug(sensor_name + " distance - " + str(self.distance_sensors[sensor_name].getValue()))
        return self.distance_sensors[sensor_name].getValue() > PROXIMITY_THRESHOLD

    def getWallFromProximity(self):
        left_distance = self.distance_sensors["left"].getValue() 
        right_distance = self.distance_sensors["right"].getValue()
        front_left_distance = self.distance_sensors["front-left"].getValue() 
        front_right_distance = self.distance_sensors["front-right"].getValue()
        left_corner_distance = self.distance_sensors["left-corner"].getValue() 
        right_corner_distance = self.distance_sensors["right-corner"].getValue()
        logging.debug("Left distance - " + str(left_distance))
        logging.debug("Right distance - " + str(right_distance))
        logging.debug("Front left distance - " + str(front_left_distance))
        logging.debug("Front right distance - " + str(front_right_distance))
        logging.debug("Left corner distance - " + str(left_corner_distance))
        logging.debug("Right corner distance - " + str(right_corner_distance))
        
        is_front_blocked = (front_left_distance > PROXIMITY_THRESHOLD) or (front_right_distance > PROXIMITY_THRESHOLD) or (left_corner_distance > PROXIMITY_THRESHOLD) or (right_corner_distance > PROXIMITY_THRESHOLD)
        is_left_blocked = left_distance > PROXIMITY_THRESHOLD
        is_right_blocked = right_distance > PROXIMITY_THRESHOLD
        return is_front_blocked, is_left_blocked, is_right_blocked

    def setWallFromCamera(self, maze_coord, facing_dir):
        # valid_colors = ["black", "green", "teal", "lime", "olive"]
        valid_colors = ["black"]
        is_wall_infront = self.color_detector.testColorInImageRow(valid_colors, CAMERA_TEST_PIXEL_ROW)
        if is_wall_infront:
            self.saveImage(0)
            self.setPathBlocked(maze_coord, facing_dir, CELL_OFFSET)
        else:
            self.saveImage(1)
        return is_wall_infront

    def getWallManually(self):
        user_input = input("Enter 0 if wall is present : ")
        if int(user_input) == 0:
            return True
        return False

    def setCellDirectionBlocked(self, cell, direction):
        updatable_cell = self.maze.getCellInDirection(cell, direction, CELL_OFFSET)
        self.maze.addDataToMaze(updatable_cell, 0)

    def saveImage(self, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(self.maze_coord) + self.orientation + str(is_not_facing_wall) + ".png", 100)

    def mapWalls(self, maze_coord, orientation):
        self.maze_coord = maze_coord
        self.orientation = orientation

        is_front_blocked, is_left_blocked, is_right_blocked = self.getWallFromProximity()
      
        front_cell = self.maze.getCellInDirection(self.maze_coord, self.orientation, CELL_OFFSET)
        if is_front_blocked: 
            logging.debug(str(self.maze_coord) + " " + self.orientation + " - Front blocked")
            print(self.maze_coord, "Front blocked")
            self.maze.addDataToMaze(front_cell, 0)
        else:
            self.maze.addDataIfUnknown(front_cell, 1)

        left_dir = self.maze.getLeftDir(self.orientation)
        right_dir = self.maze.getOppositeDir(left_dir)

        left_cell = self.maze.getCellInDirection(self.maze_coord, left_dir, CELL_OFFSET)
        if is_left_blocked: 
            logging.debug(str(self.maze_coord) + " - " + left_dir + " blocked")
            print(self.maze_coord, "Left blocked")
            self.maze.addDataToMaze(left_cell, 0)
        else:
            self.maze.addDataIfUnknown(left_cell, 1)

        right_cell = self.maze.getCellInDirection(self.maze_coord, right_dir, CELL_OFFSET)
        if is_right_blocked: 
            logging.debug(str(self.maze_coord) + " - " + right_dir + " blocked")
            print(self.maze_coord, "Right blocked")
            self.maze.addDataToMaze(right_cell, 0)
        else:
            self.maze.addDataIfUnknown(right_cell, 1)

    def findMLine(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        
        # Calculate the slope of the line
        slope = (y2 - y1) / (x2 - x1)
        
        # Calculate the y-intercept of the line
        y_intercept = y1 - slope * x1
        
        return slope, y_intercept

    def setMLine(self, start_point, goal_point):
        if not self.m_line:
            self.m_line = self.findMLine(start_point, goal_point)
        return self.m_line

    def clearMLine(self):
        self.m_line = None
        self.last_intersection_distance = float("inf")

    def intersectWithMLine(self, position, tolerance = MLINE_WIDTH / 2):
        x, y = position
        m, c = self.m_line
        distance = abs(y - m*x - c) / math.sqrt(m**2 + 1)
        return distance <= tolerance

    def isCloserToGoal(self, position, goal):
        distance = self.getDistanceBetweenPoints(position, goal)
        if self.last_intersection_distance - distance > SAME_INTERSECTION_THRESHOLD:
            self.last_intersection_distance = distance
            return True
        return False

    def getAngleToGoal(self, start, goal):
        dy = goal[1] - start[1]
        dx = goal[0] - start[0]
        angle = math.atan2(dy, dx)
        return (2*math.pi - angle) % (2*math.pi)

    def getDistanceBetweenPoints(self, start, goal):
        dy = goal[1] - start[1]
        dx = goal[0] - start[0]
        return math.sqrt(dx**2 + dy**2)

    def hasReachedGoal(self, position, goal):
        distance = self.getDistanceBetweenPoints(position, goal)
        print("Distance to goal : ", distance)
        return distance <= GOAL_REACHED_THRESHOLD

    def setLastProximityReadingTime(self, time):
        self.last_proximity_reading_time = time

    def isWandering(self, time):
        return time - self.last_proximity_reading_time > WANDERING_TIMEOUT
