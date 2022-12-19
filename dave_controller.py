import sys
sys.path.append("E:\\Program Files\\Webots\\lib\\controller\\python39")

from controller import Robot
import json, math, logging
from maze import Maze
from color_detector import ColorDetector
from slam import SLAM

logging.basicConfig(filename="log.txt", level=logging.DEBUG, format="%(asctime)s : %(levelname)s : %(message)s")

### Robot Constants ###
# MOTOR_MAX_SPEED = 6.28
MOTOR_MAX_SPEED = 3.0
# SPEED = 5.0
SPEED = 2.0
# TURN_SPEED = 1.5
TURN_SPEED = 1.0
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
TWO_PI = math.pi * 2
PI = math.pi
HALF_PI = math.pi / 2
QUARTER_PI = math.pi / 4

### PID Constants ###
PID_P = 2.5
PID_D = 0.0


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
        self.maze = Maze(GRID_SIZE, GRID_SIZE)
        self.maze.resetMaze()
        self.money_drops = []
        self.exchanger_locs = []
        self.path = None
        self.travelled_cells = set()

        # Localization variables
        self.slam = SLAM(self.maze, self.camera, self.distance_sensors, self.color_detector)
        self.robot_x_axis = 0 # x axis is the robot's vertical axis in Webots
        self.robot_y_axis = 1 # y axis is the robot's horizontal axis in Webots
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
            heading = math.radians(360 - rec_data["robotAngleDegrees"])
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
        logging.info("Turning " + direction)
        start_time = self.time
        start_cell = self.getCurrentCell()
        start_orientation = self.orientation
        if direction == "right":
            turn_dir = 1
            if start_orientation == "W":
                expected_bearing = TWO_PI
            else:
                expected_bearing = self.maze.directionToAngle[self.maze.getRightDir(start_orientation)]

        elif direction == "left":
            turn_dir = -1
            expected_bearing = self.maze.directionToAngle[self.maze.getLeftDir(start_orientation)]

        # TODO - Fix specified angle rotation 
        else:
            expected_bearing = direction
            if expected_bearing < self.position[2]:
                turn_dir = -1
            else:
                turn_dir = 1

        def shouldTurnCW(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "W" and turned_angle < PI:
                turned_angle += TWO_PI
            elif init_orien == "N" and turned_angle > PI:
                turned_angle -= TWO_PI
            return turned_angle < expected_bearing

        def shouldTurnCCW(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "E" and turned_angle > PI:
                turned_angle -= TWO_PI
            elif init_orien == "N" and turned_angle < PI:
                turned_angle += TWO_PI
            return turned_angle > expected_bearing

        def shouldTurn(turn_dir, init_orien, expected_bearing):
            if turn_dir == 1:
                return shouldTurnCW(init_orien, expected_bearing)
            else:
                return shouldTurnCCW(init_orien, expected_bearing)

        self.stopMotors()
        while shouldTurn(turn_dir, start_orientation, expected_bearing):
            self.left_motor.setVelocity(TURN_SPEED * turn_dir)
            self.right_motor.setVelocity(-TURN_SPEED * turn_dir)
            self.updateRobotData()
            if self.time - start_time > TURN_TIMEOUT:
                print("Turn timed out")
                self.stopMotors()
                # left_dir = self.maze.getLeftDir(start_orientation)
                # if direction == "left":
                #     self.slam.setPathBlocked(start_cell, left_dir)
                # elif direction == "right":
                #     right_dir = self.maze.getOppositeDir(left_dir)
                #     self.slam.setPathBlocked(start_cell, right_dir)
                break
        self.stopMotors()

    def goFoward(self, target_cell):
        logging.info("Going foward")
        start_time = self.time
        photo_time = start_time + PHOTO_TIMEOUT
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

        last_error = 0
        while shouldMove(start_orientation, stop_target):
            error = getSpeedChange(start_orientation, centerline)
            # change = error * PID_P + (error - last_error) * PID_D
            change = error * PID_P
            last_error = error

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.updateRobotData()
            # if self.time > photo_time:
            #     if self.slam.setWallFromCamera(start_cell, start_orientation):
            #         self.stopMotors()
            #         return False
            #     photo_time = self.time + PHOTO_TIMEOUT
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.debug("Moving foward - Current time " + str(self.time) + " Start time " + str(start_time))
                self.stopMotors()
                self.slam.setCellDirectionBlocked(start_cell, start_orientation)
                return False

        # self.stopMotors()
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

        last_error = 0
        while shouldMove(start_orientation, stop_target):
            error = getSpeedChange(start_orientation, centerline)
            # change = error * PID_P + (error - last_error) * PID_D
            change = error * PID_P 
            last_error = error

            self.left_motor.setVelocity(-SPEED - change)
            self.right_motor.setVelocity(-SPEED + change)
            self.updateRobotData()
            if self.time - start_time > MOVEMENT_TIMEOUT:
                logging.debug("Moving backward - Current time " + str(self.time) + " Start time " + str(start_time))
                self.stopMotors()
                self.slam.setCellDirectionBlocked(start_cell, self.maze.getOppositeDir(start_orientation))
                return False

        # self.stopMotors()
        return True

    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True

    def moveToNextCell(self, current_cell, next_cell):
        logging.info("Moving from " + str(current_cell) + " to " + str(next_cell))
        if self.orientation == "N":
            if next_cell[0] < current_cell[0]:
                return self.goFoward(next_cell)
            elif next_cell[0] > current_cell[0]:
                return self.goBackward(next_cell)
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
                return self.goBackward(next_cell)
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
                return self.goBackward(next_cell)
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
                return self.goBackward(next_cell)
            elif next_cell[0] < current_cell[0]:
                self.turn("right")
                return self.goFoward(next_cell)
            elif next_cell[0] > current_cell[0]:
                self.turn("left")
                return self.goFoward(next_cell)

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
        if GRID_SIZE % 2 == 0 :
            x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
            y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH / 2
        else:
            x = ((GRID_SIZE // 2 - maze_cell[1]) * SQUARE_LENGTH) + SQUARE_LENGTH
            y = ((GRID_SIZE // 2 - maze_cell[0]) * SQUARE_LENGTH) + SQUARE_LENGTH
        return (y, x)

    def turnAndMap(self, maze_coord):
        # directions = ["N", "E", "S", "W"]
        # cell_info = self.maze.maze_map[maze_coord]
        # walls = [cell_info[d] for d in directions]
        # k = directions.index(self.orientation)

        # right_turns = 0
        # for i in range(4):
        #     if walls[k] == -1:
        #         right_turns = i
        #     k = (k + 1) % 4
        # left_turns = 0
        # for i in range(4):
        #     if walls[k] == -1:
        #         left_turns = i
        #     k = (k - 1) % 4

        self.slam.mapWalls(maze_coord, self.orientation)
        # if right_turns > left_turns:
        #     for i in range(left_turns):
        #         self.turn("left")
        #         self.slam.mapWalls(maze_coord, self.orientation)
        # else:
        #     for i in range(right_turns):
        #         self.turn("right")
        #         self.slam.mapWalls(maze_coord, self.orientation)

    def setPath(self, current_cell):
        def getShortestPath(target_locs, return_loc):
            if return_loc:
                target_locs.append(return_loc)
                return_cell = self.worldCoordToMazeCell(return_loc)
                path = self.maze.getPath(current_cell, return_cell)
                for loc in target_locs:                
                    cell = self.worldCoordToMazeCell(loc)
                    if cell in path:
                        return path
            
            min_length = 10000
            min_path = None
            for loc in target_locs:
                cell = self.worldCoordToMazeCell(loc)
                path = self.maze.getPath(current_cell, cell)
                if len(path) < min_length:
                    min_length = len(path)
                    min_path = path

            return min_path

        # if len(self.money_drops) > 0 and self.rupees == 2000:
        #     self.path = getShortestPath(self.money_drops, self.exchanger_locs[0])
        # elif len(self.money_drops) > 0 and self.rupees < 2000:
        #     self.path = getShortestPath(self.money_drops, None)
        # elif self.rupees == CAPACITY:
        #     exchanger_cell = self.worldCoordToMazeCell(self.exchanger_locs[0])
        #     self.path = self.maze.getPath(current_cell, exchanger_cell)
        # else:
        #     self.path = self.maze.getPath(current_cell, (1,1))
        #     logging.debug("Path selection - Targetless (Learning Path Only)")

        goal = self.worldCoordToMazeCell(self.money_drops[0])
        self.path = self.maze.getPath(current_cell, goal)

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    ### Main function ###
    def run(self):
        logging.info("Starting Robot")
        state = 0

        while self.robot.step(self.timestep) != -1:
            # Robot behavior is modeled as a state machine
            self.updateRobotData()
            
            # State 0 - Use Left Hand to the Wall Algorithm to follow walls and map valid paths
            if state == 0:
                current_cell = self.getCurrentCell()
                self.maze.addDataIfUnknown(current_cell, 1)


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

            # State 1: Turn around to find walls
            elif state == 1:
                current_cell = self.getCurrentCell()
                self.travelled_cells.add(current_cell)

                self.turnAndMap(current_cell)

                state = 2

            # State 2: Find and move to goal
            elif state == 2:
                current_cell = self.getCurrentCell()
                self.setPath(current_cell)

                next_cell = self.path[current_cell]
                print("Moving from " + str(current_cell) + " to " + str(next_cell))
                if not self.moveToNextCell(current_cell, next_cell):
                    print("Error moving to cell " + str(next_cell) + " Recorded as blocked")
                    state = 1
                    continue                    

                current_cell = self.getCurrentCell()
                if current_cell not in self.travelled_cells:
                    state = 1
                else:
                    state = 2


dave = PuckController()
dave.run()
