import sys
# Python version should be 3.9.5
sys.path.append("E:\\Program Files\\Webots\\lib\\controller\\python39\\") 

from controller import Robot
import json, math, logging, time
from modules.anglr import Angle
from position import Position
from maze import Maze
from slam import SLAM
from game import Game

logging.basicConfig(filename="logs/log-" + str(round(time.time())) + ".txt", level=logging.DEBUG, format="%(asctime)s : %(levelname)s : %(message)s")
logging.logProcesses = 0
logging.logThreads = 0

### Robot Constants ###
SPEED = 4.5 # Max - 6.28
TURN_SPEED = 4.0
TURN_TIMEOUT = 2.0
MOVEMENT_TIMEOUT = 1.5
WANDER_TIME = 210
SEARCH_RADIUS = 1.0
POINT_RADIUS = 0.06 # Max - 0.085

### PID Constants ###
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
        self.camera = None
        # self.camera = self.robot.getDevice("camera")
        # self.camera.setFov(0.62)
        # self.camera.setExposure(5.0)
        # self.camera.enable(self.timestep)

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
        self.slam = SLAM(self.maze, self.camera, self.distance_sensors)
        self.position = None
        self.orientation = None

        # Game variables
        self.game = Game(self.maze)
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
            
            self.exchanger_locs = rec_data["goals"]
            self.maze.setKnownPoints(self.exchanger_locs, 1)
            self.money_drops = rec_data["collectibles"]
            if self.game_time == 0 and self.time < WANDER_TIME:
                self.maze.setKnownPoints(self.money_drops, 0)
            else:
                self.maze.setKnownPoints(self.money_drops, 1)

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
                return Angle(0) < current_angle.angleBetweenCW(target_heading) < Angle(math.pi)
            else:
                return Angle(0) < current_angle.angleBetweenCCW(target_heading) < Angle(math.pi)

        while shouldTurn(turn_dir, target_heading):
            self.left_motor.setVelocity(TURN_SPEED * turn_dir)
            self.right_motor.setVelocity(-TURN_SPEED * turn_dir)
            self.updateRobotData()
            if self.time - start_time > TURN_TIMEOUT:
                print("Turn timed out")
                self.stopMotors()
                break

    def goFoward(self, target_cell):
        logging.info("Going foward")
        start_time = self.time
        ideal_position = Maze.mazeCellToWorldCoords(target_cell)
        stop_target, centerline = self.position.getRelativeXY(ideal_position)

        def shouldStop(start_time):
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.warning("Movement timeout: Current time - " + str(self.time) + " Start time - " + str(start_time))
                self.slam.setTimeOutBlocked(target_cell, self.orientation)
                return True
            return self.slam.isFrontBlocked()

        while self.position.isTargetReachableMovingFoward(stop_target):
            error = self.position.getHeading().getSignedError(self.orientation)
            change = error * PID_P

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.updateRobotData()
            if shouldStop(start_time):
                self.stopMotors()
                return self.position.getMazeCell() == target_cell
        return True

    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True

    def moveToNextCellWithoutReverse(self, current_cell, next_cell):
        logging.info("Moving from " + str(current_cell) + " to " + str(next_cell))
        if self.orientation == "N":
            v_diff = next_cell[0] - current_cell[0]
            h_diff = next_cell[1] - current_cell[1]
        elif self.orientation == "S":
            v_diff = current_cell[0] - next_cell[0]
            h_diff = current_cell[1] - next_cell[1]
        elif self.orientation == "E":
            v_diff = current_cell[1] - next_cell[1]
            h_diff = next_cell[0] - current_cell[0] 
        elif self.orientation == "W":
            v_diff = next_cell[1] - current_cell[1]
            h_diff = current_cell[0] - next_cell[0] 

        if v_diff == -1:
            return self.goFoward(next_cell)
        elif v_diff == 1:
            self.turn("right")
            self.turn("right")
            return self.goFoward(next_cell)
        elif h_diff == 1:
            self.turn("right")
            return self.goFoward(next_cell)
        elif h_diff == -1:
            self.turn("left")
            return self.goFoward(next_cell)

    def setPath(self, current_cell):
        exchange_cells = [Maze.worldCoordToMazeCell(exchange) for exchange in self.exchanger_locs]
        exchange_rates = [exchange[2] for exchange in self.exchanger_locs]
        money_cells = [Maze.worldCoordToMazeCell(money) for money in self.money_drops]

        self.game.setData(current_cell, money_cells, exchange_cells, exchange_rates, self.rupees, self.dollars, self.time)
        goal = self.game.getGoal()
        self.path = self.maze.getPath(current_cell, goal)

    ### Main function ###
    def run(self):
        logging.info("Starting Robot")
        print("Starting Robot")
        while True:
            while not self.updateRobotData():
                logging.info("Waiting for transmitter to connect")

            current_cell = self.position.getMazeCell()
            
            self.setPath(current_cell)

            next_cell = self.path[current_cell]

            was_move_successful = self.moveToNextCellWithoutReverse(current_cell, next_cell)
            current_cell = self.position.getMazeCell()

            logging.debug("Current cell - " + str(current_cell))
            if was_move_successful:
                logging.info("Moved to cell " + str(next_cell))
            else:
                logging.info("Cannot move to cell " + str(next_cell))

            self.slam.mapWallsAutomatically(current_cell, self.orientation)

            if was_move_successful:
                self.slam.setCellAccessible(current_cell)

            self.maze.showMaze(self.path, current_cell)

dave = PuckController()
dave.run()
