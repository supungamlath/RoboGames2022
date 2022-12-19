import logging

PROXIMITY_THRESHOLD = 90
CAMERA_TEST_PIXEL_ROW = 28
CELL_OFFSET = 1 # Wall data should be offset by this constant to account for the robot's diameter 

# Class for Simultaneous Localization and Mapping
class SLAM:
    def __init__(self, maze, camera, distance_sensors, color_detector):
        self.maze = maze
        self.camera = camera
        self.distance_sensors = distance_sensors
        self.color_detector = color_detector
        self.maze_coord = None
        self.orientation = None

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