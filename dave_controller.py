import sys
sys.path.append("E:\\Program Files\\Webots\\lib\\controller\\python39")

from controller import Robot
import json, math, logging, time
from maze import Maze
from position import Position
from color_detector import ColorDetector
from slam import SLAM
from modules.anglr import Angle

logging.basicConfig(filename="logs/log-" + str(time.time()) + ".txt", level=logging.DEBUG, format="%(asctime)s : %(levelname)s : %(message)s")
logging.logProcesses = 0
logging.logThreads = 0

### Robot Constants ###
SPEED = 4.5 # Max - 6.28
TURN_SPEED = 4.0
CAPACITY = 10000
TURN_TIMEOUT = 2.0
MOVEMENT_TIMEOUT = 2.0

### Math Constants ###
TWO_PI = Angle(math.pi * 2)
PI = Angle(math.pi)

### PID Constants ###
# PID_P = 2.5
PID_P = 5.0

class PuckController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.time = 0

        # Receiver initialization
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

        # Camera initialization
        self.camera = self.robot.getDevice("camera")
        # self.camera.setFov(0.62)
        self.camera.setExposure(5.0)
        self.camera.enable(self.timestep)
        self.color_detector = ColorDetector(self.camera)

        # LED initialization
        for i in range(10):
            self.robot.getDevice("led" + str(i)).set(1)

        # Proximity sensor initialization
        self.distance_sensors = {}
        ps_names = ["front-right", "right-corner", "right", "rear-right", "rear-left", "left", "left-corner", "front-left"]
        for ind, name in enumerate(ps_names):
            self.distance_sensors[name] = self.robot.getDevice("ps" + str(ind))
            self.distance_sensors[name].enable(self.timestep)

        # Motor initialization
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Maze initialization
        self.maze = Maze()
        self.maze.resetMaze()
        self.money_drops = []
        self.exchanger_locs = []
        self.path = {}

        # Localization variables
        self.slam = SLAM(self.maze, self.camera, self.distance_sensors, self.color_detector)
        self.robot_x_axis = 0 # x axis is the robot's vertical axis in Webots
        self.robot_y_axis = 1 # y axis is the robot's horizontal axis in Webots
        self.position = None
        self.orientation = None

        # Game variables
        self.game_time = 0
        self.rupees = 0
        self.dollars = 0

    # Function to update robot attributes from received data
    def updateRobotData(self):
        self.robot.step(self.timestep)
        self.time = self.robot.getTime()
        was_data_set = False
        
        while self.receiver.getQueueLength() > 0:
            was_data_set = True
            rec_data = json.loads(self.receiver.getData().decode("utf-8"))
            self.game_time = rec_data["time"]
            self.rupees = rec_data["rupees"]
            self.dollars = rec_data["dollars"]

            x, y = rec_data["robot"]
            heading = Angle(360 - rec_data["robotAngleDegrees"], "degrees")
            self.position = Position(x, y, heading)
            self.orientation = self.position.getOrientation()
            
            money_drops = rec_data["collectibles"]
            exchanger_locs = rec_data["goals"]
            self.maze.setPointsOfInterest(money_drops + exchanger_locs, self.money_drops + self.exchanger_locs)
            self.money_drops = money_drops
            self.exchanger_locs = exchanger_locs                

            self.receiver.nextPacket()

        return was_data_set

    ### Motion Functions ###
    def turn(self, direction):
        logging.info("Turning " + direction)
        start_time = self.time
        if direction == "right":
            turn_dir = 1
            target_heading = Angle.getIdealAngleCW(self.orientation)
        elif direction == "left":
            turn_dir = -1
            target_heading = Angle.getIdealAngleCCW(self.orientation)

        def shouldTurn(turn_dir, target_heading):
            current_angle = self.position.getHeading()
            if turn_dir == 1:
                return Angle(0) < current_angle.angleBetweenCW(target_heading) < PI
            else:
                return Angle(0) < current_angle.angleBetweenCCW(target_heading) < PI

        # self.stopMotors()
        while shouldTurn(turn_dir, target_heading):
            self.left_motor.setVelocity(TURN_SPEED * turn_dir)
            self.right_motor.setVelocity(-TURN_SPEED * turn_dir)
            self.updateRobotData()
            if self.time - start_time > TURN_TIMEOUT:
                print("Turn timed out")
                # self.stopMotors()
                break
        # self.stopMotors()

    def goFoward(self, target_cell):
        logging.info("Going foward")
        start_time = self.time
        ideal_position = Maze.mazeCellToWorldCoords(target_cell)
        stop_target, centerline = self.position.getRelativeXY(ideal_position)

        def shouldStop(start_time):
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.debug("Movement timeout: Current time - " + str(self.time) + " Start time - " + str(start_time))
                self.slam.setTimeOutBlocked(target_cell)
                return True
            return self.slam.isFrontBlocked()

        while self.position.isTargetReachableMovingFoward(stop_target):
            error = self.position.getHeading().getSignedError(self.orientation)
            change = error * PID_P

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.updateRobotData()
            if shouldStop(start_time):
                # self.stopMotors()
                return self.getCurrentCell() == target_cell
        return True

    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True

    def moveToNextCellWithoutReverse(self, current_cell, next_cell):
        logging.info("Moving from " + str(current_cell) + " to " + str(next_cell))
        if self.orientation == "N":
            if next_cell[0] < current_cell[0]:
                return self.goFoward(next_cell)
            elif next_cell[0] > current_cell[0]:
                self.turn("right")
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[1] > current_cell[1]:
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[1] < current_cell[1]:
                self.turn("left")
                return self.goFoward(next_cell)
        elif self.orientation == "E":
            if next_cell[1] > current_cell[1]:
                return self.goFoward(next_cell)
            elif next_cell[1] < current_cell[1]:
                self.turn("right")
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[0] > current_cell[0]:
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[0] < current_cell[0]:
                self.turn("left")
                return self.goFoward(next_cell)
        elif self.orientation == "S":
            if next_cell[0] > current_cell[0]:
                return self.goFoward(next_cell)
            elif next_cell[0] < current_cell[0]:
                self.turn("right")
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[1] < current_cell[1]:
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[1] > current_cell[1]:
                self.turn("left")
                return self.goFoward(next_cell)
        elif self.orientation == "W":
            if next_cell[1] < current_cell[1]:
                return self.goFoward(next_cell)
            elif next_cell[1] > current_cell[1]:
                self.turn("right")
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[0] < current_cell[0]:
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[0] > current_cell[0]:
                self.turn("left")
                return self.goFoward(next_cell)

    def setPath(self, current_cell):
        def getShortestPath(target_locs, return_loc):
            if return_loc:
                target_locs.append(return_loc)
                return_cell = Maze.worldCoordToMazeCell(return_loc)
                path = self.maze.getPath(current_cell, return_cell)
                for loc in target_locs:                
                    cell = Maze.worldCoordToMazeCell(loc)
                    if cell in path:
                        return path
            
            min_length = 10000
            min_path = None
            for loc in target_locs:
                cell = Maze.worldCoordToMazeCell(loc)
                path = self.maze.getPath(current_cell, cell)
                if len(path) < min_length:
                    min_length = len(path)
                    min_path = path

            return min_path

        if len(self.money_drops) > 0 and self.rupees > 8000:
            self.path = getShortestPath(self.money_drops, self.exchanger_locs[0])
        elif len(self.money_drops) > 0 and self.rupees < 2000:
            self.path = getShortestPath(self.money_drops, None)
        elif self.rupees == CAPACITY:
            exchanger_cell = Maze.worldCoordToMazeCell(self.exchanger_locs[0])
            self.path = self.maze.getPath(current_cell, exchanger_cell)
        else:
            self.path = self.maze.getPath(current_cell, (1,1))
            logging.debug("Path selection - Targetless (Learning Path Only)")

        # goal = Maze.worldCoordToMazeCell(self.exchanger_locs[1])
        # self.path = self.maze.getPath(current_cell, goal)

    def setPathManually(self, current_cell):
        direction = input("Enter direction (N, E, S, W): ")
        direction = direction.upper()
        if direction == "N":
            self.path = {current_cell: (current_cell[0] - 1, current_cell[1])}
        elif direction == "E":
            self.path = {current_cell: (current_cell[0], current_cell[1] + 1)}
        elif direction == "S":
            self.path = {current_cell: (current_cell[0] + 1, current_cell[1])}
        elif direction == "W":
            self.path = {current_cell: (current_cell[0], current_cell[1] - 1)}
        else:
            print("Invalid direction")
            self.setPathManually(current_cell)

    def getCurrentCell(self):
        return Maze.worldCoordToMazeCell(self.position.getWorldXY())

    ### Main function ###
    def run(self):
        logging.info("Starting Robot")
        state = 1

        while True:
            # Robot behavior is modeled as a state machine
            while not self.updateRobotData():
                print("Waiting for transmitter to connect...")

            # State 2: Find and move to goal
            if state == 1:
                current_cell = self.getCurrentCell()
                
                if self.path is None:
                    state = 1
                    continue
                if current_cell not in self.path or self.slam.should_replan:                
                    self.setPath(current_cell)
                    self.slam.should_replan = False

                next_cell = self.path[current_cell]

                was_move_successful = self.moveToNextCellWithoutReverse(current_cell, next_cell)
                current_cell = self.getCurrentCell()

                logging.debug("Current cell - " + str(current_cell))
                if was_move_successful:
                    logging.info("Moved to cell " + str(next_cell))
                else:
                    logging.info("Cannot move to cell " + str(next_cell))

                if not self.maze.isCellKnown(current_cell):
                    self.slam.mapWallsAutomatically(current_cell, self.orientation)

                if was_move_successful:
                    self.slam.setCellAccessible(current_cell)

                self.maze.showMaze(self.path, current_cell)

dave = PuckController()
dave.run()
