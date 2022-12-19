import logging

PROXIMITY_THRESHOLD = 90
CAMERA_TEST_PIXEL_ROW = 28

# Class for Simultaneous Localization and Mapping
class SLAM:
    def __init__(self, maze, camera, distance_sensors):
        self.maze = maze
        self.camera = camera
        self.distance_sensors = distance_sensors
        self.maze_coord = None
        self.orientation = None

    def getWallFromProximity(self):
        left_distance = self.distance_sensors[5].getValue() 
        right_distance = self.distance_sensors[2].getValue()
        front_left_distance = self.distance_sensors[7].getValue() 
        front_right_distance = self.distance_sensors[0].getValue()
        mid_left_distance = self.distance_sensors[6].getValue() 
        mid_right_distance = self.distance_sensors[1].getValue()
        logging.debug("Left distance - " + str(left_distance))
        logging.debug("Right distance - " + str(right_distance))
        logging.debug("Front left distance - " + str(front_left_distance))
        logging.debug("Front right distance - " + str(front_right_distance))
        logging.debug("Mid left distance - " + str(mid_left_distance))
        logging.debug("Mid right distance - " + str(mid_right_distance))
        
        is_front_blocked = (front_left_distance > PROXIMITY_THRESHOLD) or (front_right_distance > PROXIMITY_THRESHOLD) or (mid_left_distance > PROXIMITY_THRESHOLD) or (mid_right_distance > PROXIMITY_THRESHOLD)
        is_left_blocked = left_distance > PROXIMITY_THRESHOLD
        is_right_blocked = right_distance > PROXIMITY_THRESHOLD
        return is_front_blocked, is_left_blocked, is_right_blocked

    def getWallFromCamera(self):
        valid_colors = ["green", "teal", "lime", "olive"]
        return self.color_detector.testColorInCameraRow(valid_colors, CAMERA_TEST_PIXEL_ROW)

    def getWallManually(self):
        user_input = input("Enter 0 if wall is present : ")
        if int(user_input) == 0:
            return True
        return False

    def updateMaze(self, cell_dir, is_not_facing_wall):
        self.maze.addDataToMaze(self.maze_coord, cell_dir, is_not_facing_wall)
        if is_not_facing_wall:
            logging.info(str(self.maze_coord) + " - No wall seen on " + self.orientation)
        else:
            logging.info(str(self.maze_coord) + " - Wall seen on " + self.orientation)

    def saveImage(self, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(self.maze_coord) + self.orientation + str(is_not_facing_wall) + ".png", 100)

    def setPathBlocked(self, maze_coord, blocked_dir):
        self.maze.addDataToMaze(maze_coord, blocked_dir, 0)

    def mapWalls(self, maze_coord, orientation):
        self.maze_coord = maze_coord
        self.orientation = orientation

        is_front_blocked, is_left_blocked, is_right_blocked = self.getWallFromProximity()
        # cell_info = self.maze.maze_map[self.maze_coord]
      
        if is_front_blocked: 
            logging.debug(str(self.maze_coord) + " " + self.orientation + " - Front blocked")
            print("Front blocked")
            self.updateMaze(self.orientation, 0)
            # self.saveImage(0)
        else:
            self.updateMaze(self.orientation, 1)
            # self.saveImage(1)

        left_dir = self.maze.getLeftDir(self.orientation)
        right_dir = self.maze.getOppositeDir(left_dir)

        if is_left_blocked: 
            self.updateMaze(left_dir, 0)
            logging.debug(str(self.maze_coord) + " - " + left_dir + " blocked")
            print("Left blocked")
        else:
            self.updateMaze(left_dir, 1)

        if is_right_blocked: 
            self.updateMaze(right_dir, 0)
            logging.debug(str(self.maze_coord) + " - " + right_dir + " blocked")
            print("Right blocked")
        else:
            self.updateMaze(right_dir, 1)