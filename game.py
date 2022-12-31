from maze import Maze
import logging

WANDER_TIME = 3 * 60
EXCHANGE_RADIUS = 100
POINT_RADIUS = 2 # Max - 0.085 / 0.02
CENTER_CELL = Maze.worldCoordToMazeCell((0, 0))
LOSS_PER_CELL = 1.5375

class Game:
    def __init__(self, maze):
        self.maze = maze
        self.state = 1
        self.current_cell = None
        self.money_cells = []
        self.exchange_cells = []
        self.exchange_rates = []
        self.rupees = 0
        self.target_exchange = None
        self.target_drop = None

    def setData(self, current_cell, money_cells, exchange_cells, exchange_rates, rupees, time):
        logging.info("Game time: " + str(time))
        logging.info("Rupees: " + str(rupees))
        print("Rupees: " + str(rupees))
        self.current_cell = current_cell
        self.money_cells = money_cells
        self.rupees = rupees
        self.time = time
        self.exchange_cells = exchange_cells
        self.exchange_rates = exchange_rates

    def getBestExchange(self):
        min_rate = min(self.exchange_rates)
        min_indices = [i for i, x in enumerate(self.exchange_rates) if x == min_rate]
        best_exchanges = []
        for i in min_indices:
            best_exchanges.append(self.exchange_cells[i])
        return sorted(best_exchanges, key=lambda x: Maze.getManhattanDistance(self.current_cell, x))[0]

    def getHighestExchange(self):
        distances = [Maze.getManhattanDistance(self.current_cell, exchange) for exchange in self.exchange_cells]
        losses = [distance * LOSS_PER_CELL for distance in distances]
        final_values = [(self.rupees - losses[i])/self.exchange_rates[i] for i in range(len(self.exchange_rates))]
        max_value_index = final_values.index(max(final_values))
        return self.exchange_cells[max_value_index]

    def getCellsOnRadius(self, cell, radius):
        cells = []
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if Maze.getManhattanDistance((0, 0), (i, j)) <= radius:
                    cells.append((cell[0] + i, cell[1] + j))
        return cells

    def isAtCell(self, cell):
        return Maze.getManhattanDistance(self.current_cell, cell) <= POINT_RADIUS

    def isInRadius(self, cell, radius):
        return Maze.getManhattanDistance(self.current_cell, cell) <= radius

    def getGoal(self):
        logging.info("State: " + str(self.state))
        print("State: " + str(self.state))
        if self.state == 1:
            if not self.target_exchange:
                self.target_exchange = self.getBestExchange()

            if self.rupees == 0 and self.time < WANDER_TIME and self.isInRadius(self.target_exchange, EXCHANGE_RADIUS):
                self.state = 2

            if self.rupees > 0 or self.time > WANDER_TIME:
                self.state = 3

            return self.target_exchange

        elif self.state == 2:

            if self.rupees > 0 or self.time > WANDER_TIME:
                self.state = 3
            elif self.isAtCell(CENTER_CELL):
                self.target_exchange =  self.getBestExchange()
                self.state = 1
            return CENTER_CELL

        elif self.state == 3:
            if self.target_drop is None or self.isAtCell(self.target_drop):
                if self.rupees >= 900:
                    self.target_exchange = self.getHighestExchange()
                    self.state = 4

                drops_in_exchange_radius = []
                for radius in range(EXCHANGE_RADIUS, EXCHANGE_RADIUS * 4, EXCHANGE_RADIUS):
                    best_exchange = self.getBestExchange()
                    drops_in_exchange_radius = [drop for drop in self.money_cells if Maze.getManhattanDistance(best_exchange, drop) <= radius]
                    if len(drops_in_exchange_radius) > 0:
                        break
                self.target_drop = sorted(drops_in_exchange_radius, key=lambda drop: len(self.maze.getPath(self.current_cell, drop)))[0]

            return self.target_drop

        elif self.state == 4:
            if self.isAtCell(self.target_exchange):
                self.target_exchange = self.getBestExchange()
                self.state = 1

            return self.target_exchange
        