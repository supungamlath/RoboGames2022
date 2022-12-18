import sys
sys.path.append("E:\\Program Files\\Webots\\lib\\controller\\python39")

from controller import Robot
import json, math, logging
from maze import Maze
from color_detector import ColorDetector

logging.basicConfig(filename="log.txt", level=logging.DEBUG, format="%(asctime)s : %(levelname)s : %(message)s")

### Robot Constants ###
MOTOR_MAX_SPEED = 6.28
SPEED = 5.0
TURN_SPEED = 1.5
CAMERA_TEST_PIXEL_ROW = 28
CAPACITY = 10000
MOVEMENT_TIMEOUT = 3

### Maze Constants ###
GRID_SIZE = 44
SQUARE_LENGTH = 0.1

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
        self.camera.enable(self.timestep)
        self.color_detector = ColorDetector(self.camera)

        # Proximity sensor initialization
        self.distance_sensors = []
        for i in range(8):
            ds = self.robot.getDevice("ps" + str(i))
            ds.enable(self.timestep)
            self.distance_sensors.append(ds)

        # Motor initialization
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Maze initialization
        self.maze = Maze(GRID_SIZE, GRID_SIZE)
        self.maze.loadMaze()
        self.money_drops = []
        self.exchanger_locs = []
        self.path = None
        self.travelled_cells = set()

        # Localization variables
        self.robot_x_axis = 0 # x axis is the robot's vertical axis in Webots
        self.robot_y_axis = 1 # y axis is the robot's horizontal axis in Webots
        self.position = [0, 0, 0]

        # Game variables
        self.game_time = 0
        self.rupees = 0
        self.dollars = 0

    ### Sensor Functions ###
    def getOrientation(self, angle):
        if angle < QUARTER_PI or angle >= (TWO_PI - QUARTER_PI):
            self.robot_x_axis = 0
            self.robot_y_axis = 1
            return "N"
        elif QUARTER_PI <= angle < (HALF_PI + QUARTER_PI):
            self.robot_x_axis = 1
            self.robot_y_axis = 0
            return "E"
        elif (HALF_PI + QUARTER_PI) <= angle < (PI + QUARTER_PI):
            self.robot_x_axis = 0
            self.robot_y_axis = 1
            return "S"
        elif (PI + QUARTER_PI) <= angle < (TWO_PI - QUARTER_PI):
            self.robot_x_axis = 1
            self.robot_y_axis = 0
            return "W"

    def getWallPresent(self):
        front_left_distance = self.distance_sensors[7].getValue() 
        front_right_distance = self.distance_sensors[0].getValue()
        logging.debug("Front left distance - " + str(front_left_distance))
        logging.debug("Front right distance - " + str(front_right_distance))
        
        # if front_distance > 120:
        #     return 0
        # valid_colors = ["green", "teal", "lime", "olive"]
        # if self.color_detector.testColorInCameraRow(valid_colors, CAMERA_TEST_PIXEL_ROW):
        #     return 0
        # return 1
        
        user_input = input("Enter 0 if wall is present : ")
        if int(user_input) == 0:
            return 0
        return 1

    def setTransmittedData(self):
        self.robot.step(self.timestep)
        self.time = self.robot.getTime()
        was_data_set = False
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
            self.receiver.nextPacket()

        return was_data_set

    ### Motion Functions ###
    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True

    def turn(self, direction):
        orientation = self.getOrientation(self.position[2])
        logging.info("Turning " + direction)
        if direction == "right":
            turn_dir = 1
            if orientation == "N":
                expected_bearing = HALF_PI
            elif orientation == "E":
                expected_bearing = PI
            elif orientation == "S":
                expected_bearing = PI + HALF_PI
            else:
                expected_bearing = TWO_PI

        elif direction == "left":
            turn_dir = -1
            if orientation == "N":
                expected_bearing = PI + HALF_PI
            elif orientation == "E":
                expected_bearing = 0
            elif orientation == "S":
                expected_bearing = HALF_PI
            else:
                expected_bearing = PI         

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
        while shouldTurn(turn_dir, orientation, expected_bearing):
            self.left_motor.setVelocity(TURN_SPEED * turn_dir)
            self.right_motor.setVelocity(-TURN_SPEED * turn_dir)
            self.setTransmittedData()
        self.stopMotors()

    def goFoward(self, target_cell):
        logging.info("Going foward")
        start_time = self.time
        orientation = self.getOrientation(self.position[2])
        logging.debug("Target cell - " + str(target_cell))
        ideal_position = self.mazeCellToWorldCoords(target_cell)
        logging.debug("Target position - " + str(ideal_position))

        stop_target = ideal_position[self.robot_x_axis]
        centerline = ideal_position[self.robot_y_axis]

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[0] > target
            elif init_orien == "S":
                return self.position[0] < target
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
        while shouldMove(orientation, stop_target):
            error = getSpeedChange(orientation, centerline)
            # change = error * PID_P + (error - last_error) * PID_D
            change = error * PID_P
            last_error = error

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.setTransmittedData()
            if self.time - start_time > MOVEMENT_TIMEOUT:
                self.stopMotors()
                return False

        # self.stopMotors()
        return True

    def goBackward(self, target_cell):
        logging.info("Going backward")
        start_time = self.time
        orientation = self.getOrientation(self.position[2])
        logging.debug("Target cell - " + str(target_cell))
        ideal_position = self.mazeCellToWorldCoords(target_cell)
        logging.debug("Target position - " + str(ideal_position))

        stop_target = ideal_position[self.robot_x_axis]
        centerline = ideal_position[self.robot_y_axis]

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[0] < target
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
        while shouldMove(orientation, stop_target):
            error = getSpeedChange(orientation, centerline)
            # change = error * PID_P + (error - last_error) * PID_D
            change = error * PID_P 
            last_error = error

            self.left_motor.setVelocity(-SPEED - change)
            self.right_motor.setVelocity(-SPEED + change)
            self.setTransmittedData()
            if self.time - start_time > MOVEMENT_TIMEOUT:
                self.stopMotors()
                return False

        # self.stopMotors()
        return True

    def moveToNextCell(self, current_cell, next_cell):
        logging.info("Moving from" + str(current_cell) + "to" + str(next_cell))
        front_dir = self.getOrientation(self.position[2])
        if front_dir == "N":
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
        elif front_dir == "E":
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
        elif front_dir == "S":
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
        elif front_dir == "W":
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

    def lookAround(self, maze_coord):
        def updateMazeAndSaveImage(cell_info):
            facing_dir = self.getOrientation(self.position[2])
            if cell_info[facing_dir] == -1:
                is_not_facing_wall = self.getWallPresent()
                self.camera.saveImage("images\\" + str(maze_coord) + facing_dir + str(is_not_facing_wall) + ".png", 100)
                self.maze.addWallToMaze(maze_coord, facing_dir, is_not_facing_wall)
                if not is_not_facing_wall:
                    logging.info(str(maze_coord) + " - Wall seen on " + facing_dir)
                else:
                    logging.info(str(maze_coord) + " - No wall seen on " + facing_dir)
            else:
                logging.info(str(maze_coord) + " - Wall known on " + facing_dir)

        directions = ["N", "E", "S", "W"]
        facing_dir = self.getOrientation(self.position[2])
        cell_info = self.maze.maze_map[maze_coord]
        walls = [cell_info[d] for d in directions]
        k = directions.index(facing_dir)

        right_turns = 0
        for i in range(4):
            if walls[k] == -1:
                right_turns = i
            k = (k + 1) % 4
        left_turns = 0
        for i in range(4):
            if walls[k] == -1:
                left_turns = i
            k = (k - 1) % 4

        updateMazeAndSaveImage(cell_info)
        if right_turns > left_turns:
            for i in range(left_turns):
                self.turn("left")
                updateMazeAndSaveImage(cell_info)
        else:
            for i in range(right_turns):
                self.turn("right")
                updateMazeAndSaveImage(cell_info)

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
            
            min_length = 1000
            min_path = None
            for loc in target_locs:
                cell = self.worldCoordToMazeCell(loc)
                path = self.maze.getPath(current_cell, cell)
                if len(path) < min_length:
                    min_length = len(path)
                    min_path = path

            return min_path

        if len(self.money_drops) > 0 and self.rupees == 2000:
            self.path = getShortestPath(self.money_drops, self.exchanger_locs[0])
        elif len(self.money_drops) > 0 and self.rupees < 2000:
            self.path = getShortestPath(self.money_drops, None)
        elif self.rupees == CAPACITY:
            exchanger_cell = self.worldCoordToMazeCell(self.exchanger_locs[0])
            self.path = self.maze.getPath(current_cell, exchanger_cell)
        else:
            self.path = self.maze.getPath(current_cell, (1,1))
            logging.debug("Path selection - Targetless (Learning Path Only)")

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    ### Main function ###
    def run(self):
        state = 1

        while self.robot.step(self.timestep) != -1:
            # Robot behavior is modeled as a state machine
            self.setTransmittedData()
            # Goal Algorithm V1 30 Minute Efficency: 0.23269213722187745
            # Goal Algorithm V2 30 Minute Efficency: 0.20543138369696537
            # Goal Algorithm V3 30 Minute Efficency: 0.2214623602572507
            # Goal Algorithm V4 30 Minute Efficency: 0.23749558147755387

            # Goal Algorithm V2 1 Hour Count: 241.666
            # Goal Algorithm V3 1 Hour Count: 255.555
            # Goal Algorithm V4 1 Hour Count: 263.888
            
            # State 1: Turn around to find walls
            if state == 1:
                current_cell = self.getCurrentCell()
                self.travelled_cells.add(current_cell)

                self.lookAround(current_cell)
                self.maze.loadMaze()

                state = 2

            # State 2: Find and move to goal
            elif state == 2:
                current_cell = self.getCurrentCell()
                self.setPath(current_cell)

                next_cell = self.path[current_cell]
                if not self.moveToNextCell(current_cell, next_cell):
                    logging.error("Error moving to cell " + str(next_cell))
                    state = 1
                    continue                    

                current_cell = self.getCurrentCell()
                if current_cell not in self.travelled_cells:
                    state = 1
                else:
                    state = 2


dave = PuckController()
dave.run()
