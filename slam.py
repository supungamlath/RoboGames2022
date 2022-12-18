import logging

PROXIMITY_THRESHOLD = 100
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
        logging.debug("Left distance - " + str(left_distance))
        logging.debug("Right distance - " + str(right_distance))
        logging.debug("Front left distance - " + str(front_left_distance))
        logging.debug("Front right distance - " + str(front_right_distance))
        
        front_distance = front_left_distance + front_right_distance / 2
        is_front_blocked = front_distance > PROXIMITY_THRESHOLD
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
        cell_info = self.maze.maze_map[self.maze_coord]
        if cell_info[self.orientation] == -1:
            self.maze.addWallToMaze(self.maze_coord, cell_dir, is_not_facing_wall)
            if not is_not_facing_wall:
                logging.info(str(self.maze_coord) + " - Wall seen on " + self.orientation)
            else:
                logging.info(str(self.maze_coord) + " - No wall seen on " + self.orientation)
        else:
            logging.info(str(self.maze_coord) + " - Wall known on " + self.orientation)

    def saveImage(self, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(self.maze_coord) + self.orientation + str(is_not_facing_wall) + ".png", 100)

    def setPathBlocked(self, maze_coord, blocked_dir):
        self.maze.addWallToMaze(maze_coord, blocked_dir, 0)

    def mapWalls(self, maze_coord, orientation):
        self.maze_coord = maze_coord
        self.orientation = orientation

        is_front_blocked, is_left_blocked, is_right_blocked = self.getWallFromProximity()
        if not is_front_blocked:
            is_front_blocked = self.getWallManually()
        else:
            print("Front blocked")
        
        if is_front_blocked: 
            self.updateMaze(self.orientation, 0)
            self.saveImage(0)
        else:
            self.updateMaze(self.orientation, 1)
            self.saveImage(1)

        directions = ["N", "E", "S", "W"]
        if is_left_blocked: 
            left_dir = directions[directions.index(self.orientation) - 1]
            self.updateMaze(left_dir, 0)
            print("Left blocked")
        if is_right_blocked: 
            right_dir_index = directions.index(self.orientation) + 1
            if right_dir_index > 3:
                right_dir_index = 0
            right_dir = directions[right_dir_index]
            self.updateMaze(right_dir, 0)
            print("Right blocked")