import sys

sys.path.append("E:\\Program Files\\Webots\\lib\\controller\\python39")

from controller import Robot
import json, math, logging
from maze import Maze
from color_detector import ColorDetector
from modules.anglr import Angle
from slam import SLAM

logging.basicConfig(
    filename="log.txt",
    level=logging.DEBUG,
    format="%(asctime)s : %(levelname)s : %(message)s",
)

### Robot Constants ###
# MOTOR_MAX_SPEED = 6.28
MOTOR_MAX_SPEED = 3.0
# SPEED = 5.0
SPEED = 2.0
# TURN_SPEED = 1.5
TURN_SPEED = 1.5
SEARCH_SPEED = 6.28
CAPACITY = 10000
TURN_TIMEOUT = 3
MOVEMENT_TIMEOUT = 1
PHOTO_TIMEOUT = 0.2

### Maze Constants ###
# Maze is 4 m by 4 m square with four 0.5 m extensions
GRID_SIZE = 222
SQUARE_LENGTH = 0.02

### Math Constants ###
TWO_PI = Angle(0)
PI = Angle(math.pi)
HALF_PI = Angle(math.pi / 2)
QUARTER_PI = Angle(math.pi / 4)

### PID Constants ###
PID_P = 2.5


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

        # Proximity sensor initialization
        self.distance_sensors = {}
        ps_names = [
            "front-right",
            "right-corner",
            "right",
            "rear-right",
            "rear-left",
            "left",
            "left-corner",
            "front-left",
        ]
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
        self.maze = Maze(GRID_SIZE, GRID_SIZE)
        self.maze.resetMaze()
        self.money_drops = []
        self.exchanger_locs = []
        self.goal = (-1.43, 0.1)
        self.path = None
        self.travelled_cells = set()

        # Localization variables
        self.slam = SLAM(
            self.maze, self.camera, self.distance_sensors, self.color_detector
        )
        self.robot_x_axis = 0  # x axis is the robot's vertical axis in Webots
        self.robot_y_axis = 1  # y axis is the robot's horizontal axis in Webots
        self.position = [0, 0, 0]
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

        def getOrientation(angle):
            if angle < QUARTER_PI or angle >= (TWO_PI - QUARTER_PI):
                return "N"
            elif QUARTER_PI <= angle < (HALF_PI + QUARTER_PI):
                return "E"
            elif (HALF_PI + QUARTER_PI) <= angle < (PI + QUARTER_PI):
                return "S"
            elif (PI + QUARTER_PI) <= angle < (TWO_PI - QUARTER_PI):
                return "W"

        while self.receiver.getQueueLength() > 0:
            was_data_set = True
            rec_data = json.loads(self.receiver.getData().decode("utf-8"))
            self.game_time = rec_data["time"]
            self.money_drops = rec_data["collectibles"]
            self.rupees = rec_data["rupees"]
            self.dollars = rec_data["dollars"]
            self.exchanger_locs = rec_data["goals"]
            self.position = rec_data["robot"]
            heading = Angle(360 - rec_data["robotAngleDegrees"], "degrees")
            self.position.append(heading)
            self.orientation = getOrientation(heading)
            if self.orientation == "N" or self.orientation == "S":
                self.robot_x_axis = 0
                self.robot_y_axis = 1
            else:
                self.robot_x_axis = 1
                self.robot_y_axis = 0

            self.receiver.nextPacket()

        return was_data_set

    ### Motion Functions ###
    def turn(self, direction):
        logging.info("Turning " + str(direction))
        start_time = self.time
        start_orientation = self.orientation
        if direction == "right":
            turn_dir = 1
            expected_bearing = self.maze.directionToAngle[
                self.maze.getRightDir(start_orientation)
            ]
            expected_bearing = Angle(expected_bearing)

        elif direction == "left":
            turn_dir = -1
            expected_bearing = self.maze.directionToAngle[
                self.maze.getLeftDir(start_orientation)
            ]
            expected_bearing = Angle(expected_bearing)

        else:
            expected_bearing = Angle(direction)
            if self.position[2].angle_between_cw(expected_bearing) < HALF_PI:
                turn_dir = 1
            else:
                turn_dir = -1


        def shouldTurn(turn_dir, expected_bearing):
            current_angle = self.position[2]
            if turn_dir == 1:
                return Angle(0) <= current_angle.angle_between_cw(expected_bearing) <= HALF_PI 
            else:
                return Angle(0) <= current_angle.angle_between_ccw(expected_bearing) <= HALF_PI

        self.stopMotors()
        while shouldTurn(turn_dir, expected_bearing):
            self.left_motor.setVelocity(TURN_SPEED * turn_dir)
            self.right_motor.setVelocity(-TURN_SPEED * turn_dir)
            self.updateRobotData()
            if self.time - start_time > TURN_TIMEOUT:
                print("Turn timed out")
                self.stopMotors()
                break
        self.stopMotors()

    def goFoward(self, target_cell):
        logging.info("Going foward")
        start_time = self.time
        start_cell = self.getCurrentCell()
        start_orientation = self.orientation
        ideal_position = self.mazeCellToWorldCoords(target_cell)

        stop_target = ideal_position[self.robot_x_axis]
        centerline = ideal_position[self.robot_y_axis]

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[0] < target
            elif init_orien == "S":
                return self.position[0] > target
            elif init_orien == "E":
                return self.position[1] > target
            elif init_orien == "W":
                return self.position[1] < target

        def getSpeedChange(init_orien, target):
            if init_orien == "N":
                error = self.position[1] - target
            elif init_orien == "S":
                error = target - self.position[1]
            elif init_orien == "E":
                error = self.position[0] - target
            elif init_orien == "W":
                error = target - self.position[0]
            return error

        while shouldMove(start_orientation, stop_target):
            error = getSpeedChange(start_orientation, centerline)
            change = error * PID_P

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.updateRobotData()
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.debug(
                    "Moving foward - Current time "
                    + str(self.time)
                    + " Start time "
                    + str(start_time)
                )
                self.stopMotors()
                self.slam.setCellDirectionBlocked(start_cell, start_orientation)
                return False
        return True

    def goBackward(self, target_cell):
        logging.info("Going backward")
        start_time = self.time
        start_cell = self.getCurrentCell()
        start_orientation = self.orientation
        ideal_position = self.mazeCellToWorldCoords(target_cell)

        stop_target = ideal_position[self.robot_x_axis]
        centerline = ideal_position[self.robot_y_axis]

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[0] > target
            elif init_orien == "S":
                return self.position[0] < target
            elif init_orien == "E":
                return self.position[1] < target
            elif init_orien == "W":
                return self.position[1] > target

        def getSpeedChange(init_orien, target):
            if init_orien == "N":
                error = self.position[1] - target
            elif init_orien == "S":
                error = target - self.position[1]
            elif init_orien == "E":
                error = self.position[0] - target
            elif init_orien == "W":
                error = target - self.position[0]
            return error * PID_P

        while shouldMove(start_orientation, stop_target):
            error = getSpeedChange(start_orientation, centerline)
            change = error * PID_P

            self.left_motor.setVelocity(-SPEED - change)
            self.right_motor.setVelocity(-SPEED + change)
            self.updateRobotData()
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.debug(
                    "Moving backward - Current time "
                    + str(self.time)
                    + " Start time "
                    + str(start_time)
                )
                self.stopMotors()
                self.slam.setCellDirectionBlocked(
                    start_cell, self.maze.getOppositeDir(start_orientation)
                )
                return False
        return True

    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    ### Maze Functions ###
    @staticmethod
    def worldCoordToMazeCell(world_coord):
        # Note that the world and maze have same x and y axes but different to usual Cartesian system
        # world_coord[0] is x, world_coord[1] is y
        # x is vertical and y is horizontal
        if GRID_SIZE % 2 == 0:
            x = (GRID_SIZE // 2) - int(world_coord[0] // SQUARE_LENGTH)
            y = (GRID_SIZE // 2) - int(world_coord[1] // SQUARE_LENGTH)
        else:
            x = (GRID_SIZE // 2 + 1) - int(world_coord[0] // (SQUARE_LENGTH / 2)) // 2
            y = (GRID_SIZE // 2 + 1) - int(world_coord[1] // (SQUARE_LENGTH / 2)) // 2
        # Maze cells are (vertical, horizontal)
        return (x, y)

    @staticmethod
    def mazeCellToWorldCoords(maze_cell):
        # Note that the world and maze have x and y axes opposite to usual Cartesian system
        # maze_cell[0] is vertical, maze_cell[1] is horizontal
        if GRID_SIZE % 2 == 0:
            x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
            y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
        else:
            x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH
            y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH
        return (y, x)

    def mapWhileMoving(self):
        current_cell = self.getCurrentCell()
        offsets = {"N": 1, "E": 1, "S": 1, "W": 1}
        offsets[self.maze.getRightDir(self.orientation)] = 2
        self.maze.addDataIfUnknown(current_cell, state=1, offsets=offsets)

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    def leftHandToWall(self):
        if self.slam.isObjectInProximity("front-left"):
            # print("Turn right in place")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(-SEARCH_SPEED)

        elif self.slam.isObjectInProximity("left-corner"):
            # print("Came too close, drive right")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED / 8)

        elif self.slam.isObjectInProximity("left"):
            # print("Drive foward")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)

        else:
            # print("Turn left")
            self.left_motor.setVelocity(SEARCH_SPEED / 8)
            self.right_motor.setVelocity(SEARCH_SPEED)


    ### Main function ###
    def run(self):
        logging.info("Starting Robot")
        state = 0

        while True:
            # Robot behavior is modeled as a state machine
            while not self.updateRobotData():
                print("Waiting for data...")

            # State 0 - Follow line
            if state == 0:
                print("State 0")
                self.slam.setMLine((self.position[0], self.position[1]), self.goal)
                
                angle_to_goal = Angle(self.slam.getAngleToGoal((self.position[0], self.position[1]), self.goal))
                if self.position[2].angle_between(angle_to_goal) > Angle(5, "degrees"):
                    self.turn(angle_to_goal)

                if not self.slam.isObjectInProximity("front-left") and not self.slam.isObjectInProximity("front-right"):
                    self.left_motor.setVelocity(SEARCH_SPEED)
                    self.right_motor.setVelocity(SEARCH_SPEED)       
                else:
                    state = 1

            # State 1: Go around obstacles
            elif state == 1:
                print("State 1")
                self.leftHandToWall()

                if self.slam.intersectWithMLine((self.position[0], self.position[1])):
                    if self.slam.isCloserToGoal((self.position[0], self.position[1]), self.goal):
                        state = 0


dave = PuckController()
dave.run()
