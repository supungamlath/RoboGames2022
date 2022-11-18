from controller import Robot
import json, csv, shutil, math, webcolors
from tempfile import NamedTemporaryFile
from maze import *

### Robot Constants ###
MOTOR_MAX_SPEED = 6.28
SPEED = 1.5
TURN_SPEED = 1.5
CAPACITY = 3000
SQUARE_LENGTH = 0.25
GYRO_CALIB = -4.26176112037897e-06
ENCODER_CALIB = 0.020002628550546703
# WHEEL_RADIUS = 0.0205
# AXLE_LENGTH = 0.052

### Math Constants ###
TWO_PI = math.pi * 2
PI = math.pi
HALF_PI = math.pi / 2
QUARTER_PI = math.pi / 4

class PuckController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
   
        # Receiver initialization
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.timestep)
   
        # Camera initialization
        self.camera = self.robot.getDevice("camera")
        self.camera.setFov(0.62)
        self.camera.enable(self.timestep)
    
        # Motor initialization
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
   
        # Position sensor initialization
        self.left_pos = self.robot.getDevice("left wheel sensor")
        self.right_pos = self.robot.getDevice("right wheel sensor")
        self.left_pos.enable(self.timestep)
        self.right_pos.enable(self.timestep)
        self.encoder_offsets = [0, 0]
    
        # Gyro initialization
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
   
        # Proximity sensor initialization
        self.distance_sensors = []
        for i in range(8):
            ds = self.robot.getDevice("ps" + str(i))
            ds.enable(self.timestep)
            self.distance_sensors.append(ds)

        # Maze initialization
        try:
            shutil.copy("saved_maze_empty.csv", "saved_maze.csv")
        except:
            pass
        self.maze = Maze(9, 8)
        self.maze.loadMaze()
        self.money_drops = []
        self.exchanger_cell = None
        self.goal = (1, 1)
        self.travelled_cells = set()
        self.position = [0.125, 1.125, 0]
        
        # Game variables
        self.rupees = 0
        self.dollars = 0

    ### Sensor Functions ###
    def getOrientation(self, angle):
        if angle < QUARTER_PI or angle >= (TWO_PI - QUARTER_PI):
            return "N"
        elif angle >= QUARTER_PI and angle < (HALF_PI + QUARTER_PI):
            return "E"
        elif angle >= (HALF_PI + QUARTER_PI) and angle < (PI + QUARTER_PI):
            return "S"
        elif angle >= (PI + QUARTER_PI) and angle < (TWO_PI - QUARTER_PI):
            return "W"

    def updatePosition(self):
        def getEncoderAngles():
            return (
                self.left_pos.getValue() - self.encoder_offsets[0],
                self.right_pos.getValue() - self.encoder_offsets[1],
            )

        def resetEncoderDistances():
            self.encoder_offsets[0] = self.left_pos.getValue()
            self.encoder_offsets[1] = self.right_pos.getValue()

        def getAngularVelocity():
            return self.gyro.getValues()[2] * GYRO_CALIB

        # Function to return displacements of left and right wheel given encoder angles
        def getWheelDisplacements(left_angle, right_angle):
            # compute displacement of left wheel in meters
            left_disp = left_angle * ENCODER_CALIB
            # compute displacement of right wheel in meters
            right_disp = right_angle * ENCODER_CALIB
            return left_disp, right_disp

        resetEncoderDistances()
        self.robot.step(self.timestep)
        
        self.position[2] += getAngularVelocity()
        if self.position[2] < 0:
            self.position[2] += TWO_PI
        elif self.position[2] > TWO_PI:
            self.position[2] = self.position[2] % TWO_PI

        left_angle, right_angle = getEncoderAngles()
        displacement = sum(getWheelDisplacements(left_angle, right_angle)) / 2
        #   // Update position vector:
        #   // Update in position along X direction
        self.position[0] -= displacement * math.sin(self.position[2])
        #   // Update in position along Y direction
        #   // robot position w.r.to Y direction
        self.position[1] -= displacement * math.cos(self.position[2])

    def getWallPresent(self, maze_coord, facing_dir):
        def closestColour(requested_colour):
            min_colours = {}
            for key, name in webcolors.html4_hex_to_names.items():
                r_c, g_c, b_c = webcolors.hex_to_rgb(key)
                rd = (r_c - requested_colour[0]) ** 2
                gd = (g_c - requested_colour[1]) ** 2
                bd = (b_c - requested_colour[2]) ** 2
                min_colours[(rd + gd + bd)] = name
            return min_colours[min(min_colours.keys())]

        image = self.camera.getImage()
        width = self.camera.getWidth()
        blue, green, red = 0, 0, 0
        for w in range(width):
            blue += self.camera.imageGetBlue(image, width, w, 30)
            green += self.camera.imageGetGreen(image, width, w, 30)
            red += self.camera.imageGetRed(image, width, w, 30)
        color_name = closestColour((red//width, green//width, blue//width))
        self.camera.saveImage("images\\" + str(maze_coord) + " " + facing_dir + " " + color_name + ".png", 100)

        if color_name in ["navy", "blue", "teal", "aqua"]:
            # Wall is present
            return 0
        # Wall is not present
        return 1

    def setTransmittedData(self):
        while self.receiver.getQueueLength() > 0:
            rec_data = json.loads(self.receiver.getData().decode("utf-8"))
            # print(rec_data)
            self.money_drops = rec_data["collectibles"]
            self.rupees = rec_data["rupees"]
            self.dollars = rec_data["dollars"]
            self.exchanger_cell = tuple(rec_data["goal"])

            self.receiver.nextPacket()

    def isWallInProximity(self):
        if (self.distance_sensors[0].getValue() > 140) and (
            self.distance_sensors[7].getValue() > 140
        ):
            return True

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

        def should_turn(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "W" and turned_angle < PI:
                turned_angle += TWO_PI
            elif init_orien == "N" and turned_angle > PI:
                turned_angle -= TWO_PI
            return turned_angle < expected_bearing

        while should_turn(orientation, expected_bearing):
            self.left_motor.setVelocity(TURN_SPEED)
            self.right_motor.setVelocity(-TURN_SPEED)
            self.updatePosition()

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

        def should_turn(init_orien, expected_bearing):
            turned_angle = self.position[2]
            if init_orien == "E" and turned_angle > PI:
                turned_angle -= TWO_PI
            elif init_orien == "N" and turned_angle < PI:
                turned_angle += TWO_PI
            return turned_angle > expected_bearing

        while should_turn(orientation, expected_bearing):
            self.left_motor.setVelocity(-TURN_SPEED)
            self.right_motor.setVelocity(TURN_SPEED)
            self.updatePosition()

        self.stopMotors()

    def goFoward(self):
        print("Going Foward")
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            target = self.position[1] - SQUARE_LENGTH
        elif orientation == "E":
            target = self.position[0] - SQUARE_LENGTH
        elif orientation == "S":
            target = self.position[1] + SQUARE_LENGTH
        elif orientation == "W":
            target = self.position[0] + SQUARE_LENGTH
        target = round(target * 8) / 8

        def should_move(init_orien, target):
            if init_orien == "N":
                return self.position[1] > target
            elif init_orien == "S":
                return self.position[1] < target
            elif init_orien == "E":
                return self.position[0] > target
            elif init_orien == "W":
                return self.position[0] < target

        while should_move(orientation, target):
            self.left_motor.setVelocity(SPEED)
            self.right_motor.setVelocity(SPEED)
            self.updatePosition()

        self.stopMotors()

    def goBackward(self):
        print("Going Backwards")
        orientation = self.getOrientation(self.position[2])
        if orientation == "N":
            target = self.position[1] + SQUARE_LENGTH
        elif orientation == "E":
            target = self.position[0] + SQUARE_LENGTH
        elif orientation == "S":
            target = self.position[1] - SQUARE_LENGTH
        elif orientation == "W":
            target = self.position[0] - SQUARE_LENGTH
        target = round(target * 8) / 8

        def should_move(init_orien, target):
            if init_orien == "N":
                return self.position[1] < target
            elif init_orien == "S":
                return self.position[1] > target
            elif init_orien == "E":
                return self.position[0] < target
            elif init_orien == "W":
                return self.position[0] > target

        while should_move(orientation, target):
            self.left_motor.setVelocity(-SPEED)
            self.right_motor.setVelocity(-SPEED)
            self.updatePosition()

        self.stopMotors()

    def moveToNextCell(self, current_cell, next_cell):
        print("Moving from", current_cell, "to", next_cell)
        front_dir = self.getOrientation(self.position[2])
        if front_dir == "N":
            if next_cell[0] < current_cell[0]:
                self.goFoward()
            elif next_cell[0] > current_cell[0]:
                self.goBackward()
            elif next_cell[1] > current_cell[1]:
                self.turnRight()
                self.goFoward()
            elif next_cell[1] < current_cell[1]:
                self.turnLeft()
                self.goFoward()
        elif front_dir == "E":
            if next_cell[1] > current_cell[1]:
                self.goFoward()
            elif next_cell[1] < current_cell[1]:
                self.goBackward()
            elif next_cell[0] > current_cell[0]:
                self.turnRight()
                self.goFoward()
            elif next_cell[0] < current_cell[0]:
                self.turnLeft()
                self.goFoward()
        elif front_dir == "S":
            if next_cell[0] > current_cell[0]:
                self.goFoward()
            elif next_cell[0] < current_cell[0]:
                self.goBackward()
            elif next_cell[1] < current_cell[1]:
                self.turnRight()
                self.goFoward()
            elif next_cell[1] > current_cell[1]:
                self.turnLeft()
                self.goFoward()
        elif front_dir == "W":
            if next_cell[1] < current_cell[1]:
                self.goFoward()
            elif next_cell[1] > current_cell[1]:
                self.goBackward()
            elif next_cell[0] < current_cell[0]:
                self.turnRight()
                self.goFoward()
            elif next_cell[0] > current_cell[0]:
                self.turnLeft()
                self.goFoward()

    ### Maze Functions ###
    def addWallToMaze(self, cell, dir, wall_present):
        def getOppositeDir(dir):
            if dir == "N":
                return "S"
            elif dir == "E":
                return "W"
            elif dir == "S":
                return "N"
            elif dir == "W":
                return "E"

        def getAdjacentCell(cell, dir):
            if dir == "N":
                return (cell[0] - 1, cell[1])
            elif dir == "E":
                return (cell[0], cell[1] + 1)
            elif dir == "S":
                return (cell[0] + 1, cell[1])
            elif dir == "W":
                return (cell[0], cell[1] - 1)

        fields = ["  cell  ", "E", "W", "N", "S"]
        tempfile = NamedTemporaryFile('w+t', newline='', delete=False)
        with open("saved_maze.csv", "r", newline="") as csvfile, tempfile:
            reader = csv.DictReader(csvfile, fieldnames=fields)
            writer = csv.DictWriter(tempfile, fieldnames=fields)

            adj_cell = getAdjacentCell(cell, dir)
            for row in reader:
                if row["  cell  "] == str(cell):
                    row[dir] = wall_present
                elif row["  cell  "] == str(adj_cell):
                    row[getOppositeDir(dir)] = wall_present

                writer.writerow(row)

        shutil.move(tempfile.name, "saved_maze.csv")

    def worldCoordToMazeCell(self, coord):
        # Note that the world and maze have switched x and y axes
        x = 4 - int(coord[0] // SQUARE_LENGTH)
        y = 5 + int(coord[1] // SQUARE_LENGTH)
        return (y, x)

    def lookAround(self, maze_coord):
        def updateMazeAndSaveImage(cell_info):
            facing_dir = self.getOrientation(self.position[2])
            if cell_info[facing_dir] == -1:
                is_not_facing_wall = self.getWallPresent(maze_coord, facing_dir)
                self.addWallToMaze(maze_coord, facing_dir, is_not_facing_wall)
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
            k = (k+1) % 4
        left_turns =  0
        for i in range(4):
            if walls[k] == -1:
                left_turns = i
            k = (k-1) % 4

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
            self.goal = (9,4)
            self.maze.solveMaze(current_cell, self.goal)

    def getCurrentCell(self):
        return self.worldCoordToMazeCell((self.position[0], self.position[1]))

    def calibrate(self):
        pass

    ### Main function ###
    def run(self):
        state = 1
        while self.robot.step(self.timestep) != -1:
            # Robot behavior is modeled as a state machine

            # State 0: Gyro and Encoder calibration
            if state == 0:
                self.calibrate()

            # State 1: Turn around to find walls
            elif state == 1:
                current_cell = self.getCurrentCell()
                self.travelled_cells.add(current_cell)
                
                self.lookAround(current_cell)
                self.maze.loadMaze()

                state = 2

            # State 2: Find and move to goal
            elif state == 2:
                self.setTransmittedData()
                current_cell = self.getCurrentCell()
                self.setPathAndGoal(current_cell)
                print(self.maze.path)

                next_cell = self.maze.path[current_cell]
                self.moveToNextCell(current_cell, next_cell)

                current_cell = self.getCurrentCell()
                if current_cell not in self.travelled_cells:
                    state = 1
                else:
                    state = 2


dave = PuckController()
dave.run()