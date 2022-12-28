import csv
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
SEARCH_SPEED = 5
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

        # LED initialization
        for i in range(10):
            self.robot.getDevice("led" + str(i)).set(1)

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

    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    def leftHandToWall(self):
        if self.slam.isObjectInCloseProximity("front-left"):
            # print("Turn right in place")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(-SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)

        elif self.slam.isObjectInCloseProximity("left-corner"):
            # print("Came too close, drive right")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(-SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)

        elif self.slam.isObjectInCloseProximity("left"):
            # print("Drive foward")
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)

        else:
            self.left_motor.setVelocity(SEARCH_SPEED / 8)
            self.right_motor.setVelocity(SEARCH_SPEED)

        if self.slam.isWandering(self.time):
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            

    def rightHandToWall(self): 
        if self.slam.isObjectInCloseProximity("front-right"):
            self.left_motor.setVelocity(-SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)
        
        elif self.slam.isObjectInCloseProximity("right-corner"):
            self.left_motor.setVelocity(-SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)
        
        elif self.slam.isObjectInCloseProximity("right"):
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            self.slam.setLastProximityReadingTime(self.time)

        else:
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED / 8)

        if self.slam.isWandering(self.time):
            self.left_motor.setVelocity(SEARCH_SPEED)
            self.right_motor.setVelocity(SEARCH_SPEED)
            
    ### Main function ###
    def run(self):
        logging.info("Starting Robot")
        last_distance = 0
        while True:
            # Robot behavior is modeled as a state machine
            while not self.updateRobotData():
                print("Waiting for data...")

            # self.leftHandToWall()
            # image = self.color_detector.getImageFromCamera()
            # out, lines = self.color_detector.findBlackLinesInGreenBg(image)
            # self.color_detector.showImage(out)
            distance = abs(0.25 - self.position[1])
            if abs(distance - last_distance) > 0.0005:
                last_distance = distance
                reading = self.distance_sensors["left"].getValue()
                with open("distance_sensor_left.csv", "a+", newline="") as file:
                    writer = csv.DictWriter(file, fieldnames=["Distance", "Reading"])
                    row = {"Distance":distance, "Reading":reading}
                    print(row)
                    writer.writerow(row)


dave = PuckController()
dave.run()
