import csv
import logging, math
from pprint import pprint
from model import Model

CLOSE_PROXIMITY_THRESHOLD = 300
MIN_PROXIMITY_READING = 150
CAMERA_TEST_PIXEL_ROW = 0
CELL_OFFSET = 0 # Wall data should be offset by this constant to account for the robot's diameter 
GOAL_VICINITY = 0.1
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

    def getDistanceReadings(self):
        readings = {}
        for sensor_name in self.distance_sensors:
            reading = self.distance_sensors[sensor_name].getValue()
            logging.debug(sensor_name + " distance - " + str(reading))
            readings[sensor_name] = reading
        return readings
    
    def isBallInCameraView(self):
        valid_colors = ["yellow", "olive"]
        return self.color_detector.testColorInImageRow(valid_colors, CAMERA_TEST_PIXEL_ROW)

    """Returns `True` if sensor reading of the given sensor is greater than `CLOSE_PROXIMITY_THRESHOLD`""" 
    def isObjectInCloseProximity(self, sensor_name):
        return CLOSE_PROXIMITY_THRESHOLD < self.distance_sensors[sensor_name].getValue() 

    def isFrontBlocked(self, position, ball_locs):
        if self.isObjectInCloseProximity("front-left") or self.isObjectInCloseProximity("front-right"):
            ball_distances = [position.getDistanceTo(ball_loc) for ball_loc in ball_locs]
            if min(ball_distances) < GOAL_VICINITY and self.isBallInCameraView():
                return False
            else:
                return True
        return False

    def saveImage(self, maze_coord, orientation, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(maze_coord) + orientation + str(is_not_facing_wall) + ".png", 100)

    def setCellAccessible(self, cell):
        self.maze.addDataIfUnknown(cell, 1, fill_size = CELL_OFFSET)

    def mapWallsManually(self, cell, orientation):
        with open("training_dataset.csv", "a+", newline="") as file:
            writer = csv.DictWriter(file, fieldnames = list(self.distance_sensors.keys()) + ["blocked_cells"])
            row = self.getDistanceReadings()
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
        row = self.getDistanceReadings()
        predicted_cells = self.model.predict(row)
        logging.debug("Predicted cells: " + str(predicted_cells))

        for b_cell in predicted_cells:
            self.should_replan = True
            offsets = {"parallel-axis":b_cell[0], "cross-axis":b_cell[1]}
            off_cell = self.maze.getCellInOffsetDirection(cell, orientation, offsets)
            self.maze.addDataIfUnknown(off_cell, 0)

