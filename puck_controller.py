# Setup Webots external controller, comment this out before submitting
import sys

sys.path.append("E:\Program Files\Webots\lib\controller\python39")

from controller import Robot
import json, csv, shutil, math, time, webcolors
from tempfile import NamedTemporaryFile
from maze_solver import *

### Constants ###
MAZE_FILENAME = "saved_maze.csv"
MOTOR_MAX_SPEED = 6.28
SPEED = 1
TURN_SPEED = 1
# WHEEL_RADIUS = 0.0205
SQUARE_LENGTH = 0.25
# AXLE_LENGTH = 0.052
GYRO_CALIB = -4.26176112037897e-06
ENCODER_CALIB = 0.020002628550546703
TWO_PI = math.pi * 2
PI = math.pi
HALF_PI = math.pi / 2
QUARTER_PI = math.pi / 4

### Initialize sensors and actuators ###
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# Receiver initialization
receiver = robot.getDevice("receiver")
receiver.enable(timestep)
# Camera initialization
camera = robot.getDevice("camera")
camera.setFov(0.62)
camera.enable(timestep)
# Motor initialization
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
right_motor.setVelocity(0)
# Position sensor initialization
left_pos = robot.getDevice("left wheel sensor")
right_pos = robot.getDevice("right wheel sensor")
left_pos.enable(timestep)
right_pos.enable(timestep)
# Gyro initialization
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
# Proximity sensor initialization
distance_sensors = []
for i in range(8):
    ds = robot.getDevice("ps" + str(i))
    ds.enable(timestep)
    distance_sensors.append(ds)


### Sensor Functions ###
def getEncoderAngles():
    return (
        left_pos.getValue() - encoder_offsets[0],
        right_pos.getValue() - encoder_offsets[1],
    )

def resetEncoderDistances():
    encoder_offsets[0] = left_pos.getValue()
    encoder_offsets[1] = right_pos.getValue()

def getAngularVelocity():
    return gyro.getValues()[2] * GYRO_CALIB

def getOrientation(angle):
    if angle < QUARTER_PI or angle >= (TWO_PI - QUARTER_PI):
        return "N"
    elif angle >= QUARTER_PI and angle < (HALF_PI + QUARTER_PI):
        return "E"
    elif angle >= (HALF_PI + QUARTER_PI) and angle < (PI + QUARTER_PI):
        return "S"
    elif angle >= (PI + QUARTER_PI) and angle < (TWO_PI - QUARTER_PI):
        return "W"

# Function to return displacements of left and right wheel given encoder angles
def getWheelDisplacements(left_angle, right_angle):
    # compute displacement of left wheel in meters
    left_disp = left_angle * ENCODER_CALIB
    # compute displacement of right wheel in meters
    right_disp = right_angle * ENCODER_CALIB
    return left_disp, right_disp

# Function to update position vector and print it to the console
def updatePosition():
    global position

    resetEncoderDistances()
    robot.step(timestep)
    #   // Update in orientation
    #   // orientation
    
    position[2] += getAngularVelocity()
    if position[2] < 0:
        position[2] += TWO_PI
    elif position[2] > TWO_PI:
        position[2] = position[2] % TWO_PI

    left_angle, right_angle = getEncoderAngles()
    displacement = sum(getWheelDisplacements(left_angle, right_angle)) / 2
    #   // Update position vector:
    #   // Update in position along X direction
    position[0] -= displacement * math.sin(position[2])
    #   // Update in position along Y direction
    #   // robot position w.r.to Y direction
    position[1] -= displacement * math.cos(position[2])

    # print("Estimated robot_x     : ", position[0])
    # print("Estimated robot_y     : ", position[1])
    # print("Estimated robot_orien : ", position[2])

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

def getWallPresent():
    image = camera.getImage()
    blue = camera.imageGetBlue(image, camera.getWidth(), 26, 30)
    green = camera.imageGetGreen(image, camera.getWidth(), 26, 30)
    red = camera.imageGetRed(image, camera.getWidth(), 26, 30)
    if blue == max(red, green, blue):
        # Wall is present
        return 0
    # Wall is not present
    return 1

def getReceiverData():
    global money_drops, rupees, dollars, exchanger
    while receiver.getQueueLength() > 0:
        rec_data = json.loads(receiver.getData().decode("utf-8"))
        # print(rec_data)
        money_drops = rec_data["collectibles"]
        rupees = rec_data["rupees"]
        dollars = rec_data["dollars"]
        exchanger = tuple(rec_data["goal"])

        receiver.nextPacket()

def getDistanceData():
    return [ds.getValue() for ds in distance_sensors]

def isWallInfront():
    if (distance_sensors[0].getValue() > 140) and (
        distance_sensors[7].getValue() > 140
    ):
        return True


### Motion Functions ###
def stopMotors():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    return True

def turnRight():
    print("Turning right")
    orientation = getOrientation(position[2])
    if orientation == "N":
        expected_bearing = HALF_PI
    elif orientation == "E":
        expected_bearing = PI
    elif orientation == "S":
        expected_bearing = PI + HALF_PI
    else:
        expected_bearing = TWO_PI

    def should_turn(init_orien, expected_bearing):
        turned_angle = position[2]
        if init_orien == "W" and turned_angle < PI:
            turned_angle += TWO_PI
        elif init_orien == "N" and turned_angle > PI:
            turned_angle -= TWO_PI
        return turned_angle < expected_bearing

    while should_turn(orientation, expected_bearing):
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(-TURN_SPEED)
        updatePosition()

    stopMotors()

def turnLeft():
    print("Turning left")
    orientation = getOrientation(position[2])
    if orientation == "N":
        expected_bearing = PI + HALF_PI
    elif orientation == "E":
        expected_bearing = 0
    elif orientation == "S":
        expected_bearing = HALF_PI
    else:
        expected_bearing = PI

    def should_turn(init_orien, expected_bearing):
        turned_angle = position[2]
        if init_orien == "E" and turned_angle > PI:
            turned_angle -= TWO_PI
        elif init_orien == "N" and turned_angle < PI:
            turned_angle += TWO_PI
        return turned_angle > expected_bearing

    while should_turn(orientation, expected_bearing):
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)
        updatePosition()

    stopMotors()

def goFoward():
    print("Going Foward")
    orientation = getOrientation(position[2])
    if orientation == "N":
        target = position[1] - SQUARE_LENGTH
    elif orientation == "E":
        target = position[0] - SQUARE_LENGTH
    elif orientation == "S":
        target = position[1] + SQUARE_LENGTH
    elif orientation == "W":
        target = position[0] + SQUARE_LENGTH
    target = round(target * 8) / 8

    def should_move(init_orien, target):
        # print(position[0], position[1], target)
        if init_orien == "N":
            return position[1] > target
        elif init_orien == "S":
            return position[1] < target
        elif init_orien == "E":
            return position[0] > target
        elif init_orien == "W":
            return position[0] < target

    while should_move(orientation, target):
        left_motor.setVelocity(SPEED)
        right_motor.setVelocity(SPEED)
        updatePosition()

    stopMotors()

def moveToNextCoord(current_coord, next_coord):
    print("Moving from", current_coord, "to", next_coord)
    front_dir = getOrientation(position[2])
    if front_dir == "N":
        if next_coord[0] < current_coord[0]:
            goFoward()
        elif next_coord[0] > current_coord[0]:
            turnRight()
            turnRight()
            goFoward()
        elif next_coord[1] > current_coord[1]:
            turnRight()
            goFoward()
        elif next_coord[1] < current_coord[1]:
            turnLeft()
            goFoward()
    elif front_dir == "E":
        if next_coord[1] > current_coord[1]:
            goFoward()
        elif next_coord[1] < current_coord[1]:
            turnRight()
            turnRight()
            goFoward()
        elif next_coord[0] > current_coord[0]:
            turnRight()
            goFoward()
        elif next_coord[0] < current_coord[0]:
            turnLeft()
            goFoward()
    elif front_dir == "S":
        if next_coord[0] > current_coord[0]:
            goFoward()
        elif next_coord[0] < current_coord[0]:
            turnRight()
            turnRight()
            goFoward()
        elif next_coord[1] < current_coord[1]:
            turnRight()
            goFoward()
        elif next_coord[1] > current_coord[1]:
            turnLeft()
            goFoward()
    elif front_dir == "W":
        if next_coord[1] < current_coord[1]:
            goFoward()
        elif next_coord[1] > current_coord[1]:
            turnRight()
            turnRight()
            goFoward()
        elif next_coord[0] < current_coord[0]:
            turnRight()
            goFoward()
        elif next_coord[0] > current_coord[0]:
            turnLeft()
            goFoward()


### Maze Functions ###
def addWallToMaze(cell, dir, wall_present):
    fields = ["  cell  ", "E", "W", "N", "S"]
    tempfile = NamedTemporaryFile('w+t', newline='', delete=False)
    with open(MAZE_FILENAME, "r", newline="") as csvfile, tempfile:
        reader = csv.DictReader(csvfile, fieldnames=fields)
        writer = csv.DictWriter(tempfile, fieldnames=fields)

        adj_cell = getAdjacentCell(cell, dir)
        for row in reader:
            if row["  cell  "] == str(cell):
                row[dir] = wall_present
            elif row["  cell  "] == str(adj_cell):
                row[getOppositeDir(dir)] = wall_present

            writer.writerow(row)

    shutil.move(tempfile.name, MAZE_FILENAME)

def worldToMazeCoords(coord):
    # Note that the world and maze have switched x and y axes
    x = 4 - int(coord[0] // SQUARE_LENGTH)
    y = 5 + int(coord[1] // SQUARE_LENGTH)
    return (y, x)

def lookAround(maze_coord):
    for i in range(4):
        is_not_facing_wall = getWallPresent()
        facing_dir = getOrientation(position[2])
        addWallToMaze(maze_coord, facing_dir, is_not_facing_wall)
        if not is_not_facing_wall:
            print("Wall seen on", facing_dir)
        else:
            print("No wall seen on", facing_dir)
        turnLeft()

def getGoalCell():
    if len(money_drops) > 0:
        return worldToMazeCoords(tuple(money_drops[0]))
    else:
        return (1,1)

def closestColour(requested_colour):
    min_colours = {}
    for key, name in webcolors.html4_hex_to_names.items():
        r_c, g_c, b_c = webcolors.hex_to_rgb(key)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        min_colours[(rd + gd + bd)] = name
    return min_colours[min(min_colours.keys())]

def test():
    # turnLeft()
    turnLeft()
    goFoward()
    turnLeft()
    goFoward()
    # goFoward()

    image = camera.getImage()
    blue = camera.imageGetBlue(image, camera.getWidth(), 26, 30)
    green = camera.imageGetGreen(image, camera.getWidth(), 26, 30)
    red = camera.imageGetRed(image, camera.getWidth(), 26, 30)
    print("Blue:", blue, "Green:", green, "Red:", red)
    named_color = closestColour((red,green,blue))
    print(named_color)
    
    time.sleep(10000)


### Global variables ###
money_drops = []
exchanger = None
rupees = 0
dollars = 0
path = {}
goal = getGoalCell()
travelled_cells = set()
position = [0.125, 0.375, 0]
encoder_offsets = [0, 0]
m = maze(9, 8)

### Main function ###
if __name__ == "__main__":
    state = 0
    while robot.step(timestep) != -1:
        # Robot behavior is modeled as a state machine
        for i in range(10):
            updatePosition()

        # State 0: Idle
        if state == 0:
            test()
            # if exchanger:
            #     exchanger_loc = worldToMazeCoords(exchanger)
            #     print("exchanger_loc", exchanger_loc)
            # for drop in money_drops:
            #     drop_loc = worldToMazeCoords(drop)
            #     print("drop_loc", drop_loc)

        # State 1: Turn around to find walls
        elif state == 1:
            current_coord = worldToMazeCoords((position[0], position[1]))
            travelled_cells.add(current_coord)
            
            lookAround(current_coord)
            m.solveMaze(start=current_coord, goal=goal, path=MAZE_FILENAME)
            print(m.path)

            state = 2

        # State 2: Move to money drop
        elif state == 2:
            getReceiverData()
            goal = getGoalCell()
            current_coord = worldToMazeCoords((position[0], position[1]))
            m.solveMaze(start=current_coord, goal=goal, path=MAZE_FILENAME)
            next_coord = m.path[current_coord]
            moveToNextCoord(current_coord, next_coord)

            current_coord = worldToMazeCoords((position[0], position[1]))
            if current_coord not in travelled_cells:
                state = 1
            else:
                state = 2


        # State 3: Move to exchanger
