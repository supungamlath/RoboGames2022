import csv
import logging, math
from pprint import pprint
from model import Model

HIT_PROXIMITY_THRESHOLD = 800
CLOSE_PROXIMITY_THRESHOLD = 300
MIN_PROXIMITY_READING = 150
CAMERA_TEST_PIXEL_ROW = 28
CELL_OFFSET = 0 # Wall data should be offset by this constant to account for the robot's diameter 
GOAL_REACHED_THRESHOLD = 0.1
SQUARE_LENGTH = 0.02

# Class for Simultaneous Localization and Mapping
class SLAM:
    def __init__(self, maze, camera, distance_sensors, color_detector):
        self.maze = maze
        self.camera = camera
        self.distance_sensors = distance_sensors
        self.color_detector = color_detector
        self.should_replan = True
        self.model = Model()

    def logDistances(self):
        readings = {}
        for sensor_name in self.distance_sensors:
            reading = self.distance_sensors[sensor_name].getValue()
            logging.debug(sensor_name + " distance - " + str(reading))
            readings[sensor_name] = reading
        return readings

    """Returns `True` if sensor reading of the given sensor is greater than `HIT_PROXIMITY_THRESHOLD`""" 
    def isObjectInHitProximity(self, sensor_name):
        return HIT_PROXIMITY_THRESHOLD < self.distance_sensors[sensor_name].getValue() 

    """Returns `True` if sensor reading of the given sensor is greater than `CLOSE_PROXIMITY_THRESHOLD`""" 
    def isObjectInCloseProximity(self, sensor_name):
        return CLOSE_PROXIMITY_THRESHOLD < self.distance_sensors[sensor_name].getValue() 
    
    """Returns `True` if sensor reading of the given sensor is in between `MIN_PROXIMITY_READING` and `CLOSE_PROXIMITY_THRESHOLD`""" 
    def isObjectInFarProximity(self, sensor_name):
        return MIN_PROXIMITY_READING < self.distance_sensors[sensor_name].getValue() < CLOSE_PROXIMITY_THRESHOLD

    def getCalibDistanceInCells(self, sensor_name):
        r = self.distance_sensors[sensor_name].getValue()
        if r < MIN_PROXIMITY_READING:
            return None
        distance = (-5e-11)*r**3 + (1e-07)*r**2 - 0.0001*r + 0.0933
        logging.debug(sensor_name + " distance - " + str(distance))
        return round(distance / SQUARE_LENGTH)

    def getCalibDistanceInAxes(self, distance, sensor_name):
        if sensor_name == "front-left" or sensor_name == "front-right":
            sensor_offset = 0.03162277660168379
            c = distance / sensor_offset
            return round(c * 0.03), round(c * 0.01)

        if sensor_name == "left-corner" or sensor_name == "right-corner":
            sensor_offset = 0.03330165161069343
            c = distance / sensor_offset
            return round(c * 0.022), round(c * 0.025)

        if sensor_name == "rear-left" or sensor_name == "rear-right":
            sensor_offset = 0.03354101966249685
            c = distance / sensor_offset
            return round(c * 0.03), round(c * 0.015)

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

    def saveImage(self, maze_coord, orientation, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(maze_coord) + self.orientation + str(is_not_facing_wall) + ".png", 100)

    def mapWallsLeftAndRight(self, cell, orientation):
        if self.isObjectInCloseProximity("left"):
            left_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":0, "cross-axis":2})
            self.maze.addDataIfUnknown(left_cell, 0)
            self.should_replan = True
        
        elif self.isObjectInFarProximity("left"):
            left_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":0, "cross-axis":3})
            self.maze.addDataIfUnknown(left_cell, 0)
            self.should_replan = True

        if self.isObjectInCloseProximity("right"):
            right_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":0, "cross-axis":-2})
            self.maze.addDataIfUnknown(right_cell, 0)
            self.should_replan = True

        elif self.isObjectInFarProximity("right"):
            right_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":0, "cross-axis":-3})
            self.maze.addDataIfUnknown(right_cell, 0)
            self.should_replan = True

        if self.isObjectInHitProximity("left-corner"):
            left_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":1, "cross-axis":1})
            self.maze.addDataIfUnknown(left_corner_cell, 0)
            self.should_replan = True

        elif self.isObjectInCloseProximity("left-corner"):
            left_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":2})
            self.maze.addDataIfUnknown(left_corner_cell, 0)
            self.should_replan = True

        elif self.isObjectInFarProximity("left-corner"):
            left_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":3, "cross-axis":3})
            self.maze.addDataIfUnknown(left_corner_cell, 0)
            self.should_replan = True

        if self.isObjectInHitProximity("right-corner"):
            right_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":1, "cross-axis":-1})
            self.maze.addDataIfUnknown(right_corner_cell, 0)
            self.should_replan = True

        elif self.isObjectInCloseProximity("right-corner"):
            right_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":-2})
            self.maze.addDataIfUnknown(right_corner_cell, 0)
            self.should_replan = True

        elif self.isObjectInFarProximity("right-corner"):
            right_corner_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":3, "cross-axis":-3})
            self.maze.addDataIfUnknown(right_corner_cell, 0)
            self.should_replan = True

    def mapWallsAccessible(self, cell, orientation):
        self.maze.addDataIfUnknown(cell, CELL_OFFSET, fill_size = 1)

        if self.isObjectInHitProximity("front-left"):
            front_left_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":1})
            self.maze.addDataIfUnknown(front_left_cell, 0)
            self.should_replan = True

        elif self.isObjectInCloseProximity("front-left"):
            front_left_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":3, "cross-axis":1})
            self.maze.addDataIfUnknown(front_left_cell, 0)
            self.should_replan = True
            
        if self.isObjectInHitProximity("front-right"):
            front_right_cell_1 = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":-1})
            self.maze.addDataIfUnknown(front_right_cell_1, 0)
            self.should_replan = True

        elif self.isObjectInCloseProximity("front-right"):
            front_right_cell_1 = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":3, "cross-axis":-1})
            self.maze.addDataIfUnknown(front_right_cell_1, 0)
            self.should_replan = True

        self.mapWallsLeftAndRight(cell, orientation)

    def mapWallsInaccessible(self, cell, orientation):
        # front_cell = self.maze.getCellInOffsetDirection(start_cell, orientation, {"parallel-axis":3, "cross-axis":0})
        # self.maze.addDataIfUnknown(front_cell, 0)
        # self.should_replan = True

        if self.isObjectInHitProximity("front-left"):
            front_left_cell = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":1})
            self.maze.addDataIfUnknown(front_left_cell, 0)
            self.should_replan = True

        if self.isObjectInHitProximity("front-right"):
            front_right_cell_1 = self.maze.getCellInOffsetDirection(cell, orientation, {"parallel-axis":2, "cross-axis":-1})
            self.maze.addDataIfUnknown(front_right_cell_1, 0)
            self.should_replan = True

        self.mapWallsLeftAndRight(cell, orientation)

    def setCellAccessible(self, cell):
        self.maze.addDataIfUnknown(cell, 1, fill_size = CELL_OFFSET)

    def mapWallsManually(self, cell, orientation):
        with open("training_dataset.csv", "a+", newline="") as file:
            writer = csv.DictWriter(file, fieldnames = list(self.distance_sensors.keys()) + ["blocked_cells"])
            row = self.logDistances()
            pprint(row)

            relative_blocked_cells = []
            if max(row.values()) >= MIN_PROXIMITY_READING:
                self.should_replan = True
                predicted_cells = self.model.predict(row)
                print("Predicted cells: ", predicted_cells)
                inp = input("Enter relative positions of blocked cells (comma separated). \nEnter 'a' to accept predictions. \nEnter 'n' for next cell.\n")
                while True:
                    if inp == 'n':
                        break
                    elif inp == 'a':
                        relative_blocked_cells += predicted_cells
                    else:
                        try:
                            inp = inp.split(',')
                            relative_blocked_cells.append((int(inp[0]), int(inp[1])))
                        except:
                            print("Invalid input")
                    inp = input()

                for b_cell in relative_blocked_cells:
                    offsets = {"parallel-axis":b_cell[0], "cross-axis":b_cell[1]}
                    off_cell = self.maze.getCellInOffsetDirection(cell, orientation, offsets)
                    self.maze.addDataIfUnknown(off_cell, 0)

            row["blocked_cells"] = relative_blocked_cells
            writer.writerow(row)
            self.model.trainModel()
            self.model.testModel()

    def mapWallsAutomatically(self, cell, orientation):
        row = self.logDistances()
        predicted_cells = self.model.predict(row)
        logging.debug("Predicted cells: " + str(predicted_cells))

        for b_cell in predicted_cells:
            self.should_replan = True
            offsets = {"parallel-axis":b_cell[0], "cross-axis":b_cell[1]}
            off_cell = self.maze.getCellInOffsetDirection(cell, orientation, offsets)
            self.maze.addDataIfUnknown(off_cell, 0)


    def getDistanceBetweenPoints(self, start, goal):
        dy = goal[1] - start[1]
        dx = goal[0] - start[0]
        return math.sqrt(dx**2 + dy**2)

    def hasReachedGoal(self, position, goal):
        distance = self.getDistanceBetweenPoints(position, goal)
        print("Distance to goal : ", distance)
        return distance <= GOAL_REACHED_THRESHOLD
