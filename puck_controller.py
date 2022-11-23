# import sys
# sys.path.append("E:\Program Files\Webots\lib\controller\python39")

from controller import Robot
import json, math
from maze import Maze
from color_detector import ColorDetector

### Robot Constants ###
MOTOR_MAX_SPEED = 6.28
SPEED = 5.0
TURN_SPEED = 1.5
CAPACITY = 3000
SQUARE_LENGTH = 0.25
CAMERA_TEST_ROW = 28

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

        # Receiver initialization
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)

        # Camera initialization
        self.camera = self.robot.getDevice("camera")
        # self.camera.setFov(0.62)
        self.camera.enable(self.timestep)
        self.color_detector = ColorDetector(self.camera)

        # Motor initialization
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Maze initialization
        self.maze = Maze(9, 8)
        self.maze.loadMaze()
        self.money_drops = []
        self.exchanger_cell = None
        self.goal = (1, 1)
        self.travelled_cells = set()
        self.position = [0, 0, 0]

        # Game variables
        self.time = 0
        self.rupees = 0
        self.dollars = 0

    ### Sensor Functions ###
    def getOrientation(self, angle):
        if angle < QUARTER_PI or angle >= (TWO_PI - QUARTER_PI):
            return "N"
        elif QUARTER_PI <= angle < (HALF_PI + QUARTER_PI):
            return "E"
        elif (HALF_PI + QUARTER_PI) <= angle < (PI + QUARTER_PI):
            return "S"
        elif (PI + QUARTER_PI) <= angle < (TWO_PI - QUARTER_PI):
            return "W"

    def getWallPresent(self):
        valid_colors = ["navy", "blue", "teal", "aqua"]
        if self.color_detector.testColorInCameraRow(valid_colors, CAMERA_TEST_ROW):
            return 0
        return 1

    def setTransmittedData(self):
        self.robot.step(self.timestep)
        was_data_set = False
        while self.receiver.getQueueLength() > 0:
            was_data_set = True
            rec_data = json.loads(self.receiver.getData().decode("utf-8"))
            self.time = rec_data["time"]
            self.money_drops = rec_data["collectibles"]
            self.rupees = rec_data["rupees"]
            self.dollars = rec_data["dollars"]
            self.exchanger_cell = tuple(rec_data["goal"])
            self.position = rec_data["robot"]
            in_angle = math.radians(rec_data["robotAngleDegrees"])
            heading = (PI + HALF_PI) - in_angle
            if heading < 0:
                heading += TWO_PI
            self.position.append(heading)
            # print(self.position)
            self.receiver.nextPacket()

        return was_data_set

    ### Motion Functions ###
    def stopMotors(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True

    def turnRight(self):
        print("Turning right")
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            expected_bearing = HALF_PI
        elif orientation == "E":
            expected_bearing = PI
        elif orientation == "S":
            expected_bearing = PI + HALF_PI
        else:
            expected_bearing = TWO_PI

        def shouldTurn(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "W" and turned_angle < PI:
                turned_angle += TWO_PI
            elif init_orien == "N" and turned_angle > PI:
                turned_angle -= TWO_PI
            return turned_angle < expected_bearing

        self.stopMotors()
        while shouldTurn(orientation, expected_bearing):
            self.left_motor.setVelocity(TURN_SPEED)
            self.right_motor.setVelocity(-TURN_SPEED)
            self.setTransmittedData()

        self.stopMotors()

    def turnLeft(self):
        print("Turning left")
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            expected_bearing = PI + HALF_PI
        elif orientation == "E":
            expected_bearing = 0
        elif orientation == "S":
            expected_bearing = HALF_PI
        else:
            expected_bearing = PI

        def shouldTurn(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "E" and turned_angle > PI:
                turned_angle -= TWO_PI
            elif init_orien == "N" and turned_angle < PI:
                turned_angle += TWO_PI
            return turned_angle > expected_bearing

        self.stopMotors()
        while shouldTurn(orientation, expected_bearing):
            self.left_motor.setVelocity(-TURN_SPEED)
            self.right_motor.setVelocity(TURN_SPEED)
            self.setTransmittedData()

        self.stopMotors()

    def goFoward(self):
        print("Going foward")
        start_time = self.time
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            stop_target = self.position[1] - SQUARE_LENGTH
            centerline = self.position[0]
        elif orientation == "E":
            stop_target = self.position[0] - SQUARE_LENGTH
            centerline = self.position[1]
        elif orientation == "S":
            stop_target = self.position[1] + SQUARE_LENGTH
            centerline = self.position[0]
        elif orientation == "W":
            stop_target = self.position[0] + SQUARE_LENGTH
            centerline = self.position[1]
        stop_target = round(stop_target * 8) / 8
        centerline = round(centerline * 8) / 8

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[1] > target
            elif init_orien == "S":
                return self.position[1] < target
            elif init_orien == "E":
                return self.position[0] > target
            elif init_orien == "W":
                return self.position[0] < target

        def getSpeedChange(init_orien, target):
            if init_orien == "N":
                error = self.position[0] - target
            elif init_orien == "S":
                error = target - self.position[0]
            elif init_orien == "E":
                error = target - self.position[1]
            elif init_orien == "W":
                error = self.position[1] - target
            return error

        last_error = 0
        while shouldMove(orientation, stop_target):
            error = getSpeedChange(orientation, centerline)
            change = error * PID_P + (error - last_error) * PID_D
            last_error = error

            self.left_motor.setVelocity(SPEED + change)
            self.right_motor.setVelocity(SPEED - change)
            self.setTransmittedData()
            if self.time - start_time > 5000:
                self.stopMotors()
                return False

        # self.stopMotors()
        return True

    def goBackward(self):
        print("Going backward")
        start_time = self.time
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            target = self.position[1] + SQUARE_LENGTH
            centerline = self.position[0]
        elif orientation == "E":
            target = self.position[0] + SQUARE_LENGTH
            centerline = self.position[1]
        elif orientation == "S":
            target = self.position[1] - SQUARE_LENGTH
            centerline = self.position[0]
        elif orientation == "W":
            target = self.position[0] - SQUARE_LENGTH
            centerline = self.position[1]
        target = round(target * 8) / 8
        centerline = round(centerline * 8) / 8

        def shouldMove(init_orien, target):
            if init_orien == "N":
                return self.position[1] < target
            elif init_orien == "S":
                return self.position[1] > target
            elif init_orien == "E":
                return self.position[0] < target
            elif init_orien == "W":
                return self.position[0] > target

        def getSpeedChange(init_orien, target):
            if init_orien == "N":
                error = self.position[0] - target
            elif init_orien == "S":
                error = target - self.position[0]
            elif init_orien == "E":
                error = target - self.position[1]
            elif init_orien == "W":
                error = self.position[1] - target
            return error * PID_P

        last_error = 0
        while shouldMove(orientation, target):
            error = getSpeedChange(orientation, centerline)
            change = error * PID_P + (error - last_error) * PID_D
            last_error = error

            self.left_motor.setVelocity(-SPEED - change)
            self.right_motor.setVelocity(-SPEED + change)
            self.setTransmittedData()
            if self.time - start_time > 5000:
                self.stopMotors()
                return False

        # self.stopMotors()
        return True

    def moveToNextCell(self, current_cell, next_cell):
        print("Moving from", current_cell, "to", next_cell)
        front_dir = self.getOrientation(self.position[2])
        if front_dir == "N":
            if next_cell[0] < current_cell[0]:
                return self.goFoward()
            elif next_cell[0] > current_cell[0]:
                return self.goBackward()
            elif next_cell[1] > current_cell[1]:
                self.turnRight()
                return self.goFoward()
            elif next_cell[1] < current_cell[1]:
                self.turnLeft()
                return self.goFoward()
        elif front_dir == "E":
            if next_cell[1] > current_cell[1]:
                return self.goFoward()
            elif next_cell[1] < current_cell[1]:
                return self.goBackward()
            elif next_cell[0] > current_cell[0]:
                self.turnRight()
                return self.goFoward()
            elif next_cell[0] < current_cell[0]:
                self.turnLeft()
                return self.goFoward()
        elif front_dir == "S":
            if next_cell[0] > current_cell[0]:
                return self.goFoward()
            elif next_cell[0] < current_cell[0]:
                return self.goBackward()
            elif next_cell[1] < current_cell[1]:
                self.turnRight()
                return self.goFoward()
            elif next_cell[1] > current_cell[1]:
                self.turnLeft()
                return self.goFoward()
        elif front_dir == "W":
            if next_cell[1] < current_cell[1]:
                return self.goFoward()
            elif next_cell[1] > current_cell[1]:
                return self.goBackward()
            elif next_cell[0] < current_cell[0]:
                self.turnRight()
                return self.goFoward()
            elif next_cell[0] > current_cell[0]:
                self.turnLeft()
                return self.goFoward()

    ### Maze Functions ###
    def worldCoordToMazeCell(self, coord):
        # Note that the world and maze have switched x and y axes
        x = 4 - int(coord[0] // SQUARE_LENGTH)
        y = 5 + int(coord[1] // SQUARE_LENGTH)
        return (y, x)

    def lookAround(self, maze_coord):
        def updateMazeAndSaveImage(cell_info):
            facing_dir = self.getOrientation(self.position[2])
            if cell_info[facing_dir] == -1:
                is_not_facing_wall = self.getWallPresent()
                self.maze.addWallToMaze(maze_coord, facing_dir, is_not_facing_wall)
                if not is_not_facing_wall:
                    print("Wall seen on", facing_dir)
                else:
                    print("No wall seen on", facing_dir)
            else:
                print("Wall known on", facing_dir)

        directions = ["N", "E", "S", "W"]
        facing_dir = self.getOrientation(self.position[2])
        cell_info = self.maze.maze_map[maze_coord]
        walls = [cell_info[d] for d in directions]
        # print(walls)
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
                self.turnLeft()
                updateMazeAndSaveImage(cell_info)
        else:
            for i in range(right_turns):
                self.turnRight()
                updateMazeAndSaveImage(cell_info)

    def setPathAndGoal(self, current_cell):
        if self.rupees == CAPACITY:
            self.goal = self.worldCoordToMazeCell(self.exchanger_cell)
            self.maze.solveMaze(current_cell, self.goal)

        elif len(self.money_drops) > 0:
            paths = []
            drops = []
            for drop in self.money_drops:
                drop_cell = self.worldCoordToMazeCell(drop)
                if current_cell == drop_cell:
                    continue
                self.maze.solveMaze(current_cell, drop_cell)
                paths.append(self.maze.path)
                drops.append(drop_cell)

            if len(paths) == 0:
                self.goal = self.worldCoordToMazeCell(self.exchanger_cell)
                self.maze.solveMaze(current_cell, self.goal)
                return

            self.maze.path = min(paths, key=len)
            self.goal = drops[paths.index(self.maze.path)]

        else:
            self.goal = (9, 4)
            self.maze.solveMaze(current_cell, self.goal)

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    ### Main function ###
    def run(self):
        state = 1

        while self.robot.step(self.timestep) != -1:
            # Robot behavior is modeled as a state machine
            self.setTransmittedData()

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
                self.setPathAndGoal(current_cell)
                # print(self.maze.path)

                next_cell = self.maze.path[current_cell]
                if not self.moveToNextCell(current_cell, next_cell):
                    print("Error moving to cell", next_cell)

                current_cell = self.getCurrentCell()
                if current_cell not in self.travelled_cells:
                    state = 1
                else:
                    state = 2


dave = PuckController()
dave.run()
