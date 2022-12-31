import csv
import logging, math
from pprint import pprint
from model import Model
from color_detector import ColorDetector

PROXIMITY_THRESHOLD = 300
MIN_PROXIMITY_READING = 130
CAMERA_TEST_PIXEL_ROW = 2

# Class for Simultaneous Localization and Mapping
class SLAM:
    def __init__(self, maze, camera, distance_sensors):
        self.maze = maze
        self.camera = camera
        self.distance_sensors = distance_sensors
        self.should_replan = True
        self.model = Model()
        self.color_detector = ColorDetector(self.camera)

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
    def isObjectInProximity(self, sensor_name):
        return PROXIMITY_THRESHOLD < self.distance_sensors[sensor_name].getValue() 

    def isFrontBlocked(self):
        return self.isObjectInProximity("front-left") or self.isObjectInProximity("front-right")

    def setTimeOutBlocked(self, cell):
        self.maze.addDataToMaze(cell, 0)   

    def saveImage(self, maze_coord, orientation, is_not_facing_wall):
        self.camera.saveImage("images\\" + str(maze_coord) + orientation + str(is_not_facing_wall) + ".png", 100)

    def setCellAccessible(self, cell):
        self.maze.addDataIfUnknown(cell, 1, fill_size = 0)

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
                inp = input("Enter relative positions of blocked cells (comma separated). \nEnter 'a' to accept predictions. \nEnter 'q' for next cell.\n")
                while True:
                    if inp == 'q':
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

                if len(relative_blocked_cells) > 0:
                    for b_cell in relative_blocked_cells:
                        offsets = {"parallel-axis":b_cell[0], "cross-axis":b_cell[1]}
                        off_cell = self.maze.getCellInOffsetDirection(cell, orientation, offsets)
                        self.maze.addDataIfUnknown(off_cell, 0)

            row["blocked_cells"] = relative_blocked_cells
            writer.writerow(row)
            self.model.trainModel()
            self.model.testModel()

    def mapWallsAutomatically(self, cell, orientation):
        distances = self.getDistanceReadings()
        predicted_cells = self.model.predict(distances)
        logging.debug("Predicted cells: " + str(predicted_cells))

        for b_cell in predicted_cells:
            self.should_replan = True
            offsets = {"parallel-axis":b_cell[0], "cross-axis":b_cell[1]}
            off_cell = self.maze.getCellInOffsetDirection(cell, orientation, offsets)
            self.maze.addDataIfUnknown(off_cell, 0)

