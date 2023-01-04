from maze import Maze
import logging

###
# Test 1
# Time -  mins
# Dollars -  
# Rupees - 
###

WANDER_TIME = 280.0
POINT_RADIUS = 2 # Max - 0.085 / 0.02
REALLY_CLOSE_RADIUS = 20
CLOSE_RADIUS = 50
EXCHANGE_RADIUS = 20
REACHABLE_RADIUS = 120
GOLDEN_RATE = 150.0
CENTER_CELL = Maze.worldCoordToMazeCell((0, 0))
LOSS_PER_CELL = 1.8

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
        self.exchanges_changed = False

    def setData(self, current_cell, money_cells, exchange_cells, exchange_rates, rupees, dollars, time):
        logging.debug("Dollars " + str(dollars))
        logging.debug("Rupees: " + str(rupees))
        logging.debug("Game time: " + str(time))
        self.current_cell = current_cell
        self.money_cells = money_cells
        self.rupees = rupees
        self.time = time
        if self.exchange_rates != exchange_rates:
            self.exchanges_changed = True
            print("Exchange rates " + str(exchange_rates))
            print("Dollars " + str(dollars))
            print("Rupees: " + str(rupees))
            print("Game time: " + str(time/60))
        else:
            self.exchanges_changed = False
        self.exchange_cells = exchange_cells
        self.exchange_rates = exchange_rates

    def getBestRateExchange(self):
        min_rate = min(self.exchange_rates)
        min_indices = [i for i, x in enumerate(self.exchange_rates) if x == min_rate]
        best_exchanges = []
        for i in min_indices:
            best_exchanges.append(self.exchange_cells[i])
        return min(best_exchanges, key=lambda x: Maze.getManhattanDistance(self.current_cell, x), default=None)

    def getHighestValueExchange(self):
        distances = [self.maze.getPathLength(self.current_cell, exchange) for exchange in self.exchange_cells]
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

    def getReallyCloseDrop(self, cells):
        for cell in cells:
            if self.isInRadius(cell, REALLY_CLOSE_RADIUS):
                if self.maze.getPathLength(self.current_cell, cell) <= REALLY_CLOSE_RADIUS:
                    return cell
        return None

    def getGoldenExchange(self):
        min_indices = [i for i, x in enumerate(self.exchange_rates) if x <= GOLDEN_RATE]
        sel_exchanges = []
        for i in min_indices:
            if self.isInRadius(self.exchange_cells[i], REACHABLE_RADIUS):
                sel_exchanges.append(self.exchange_cells[i])
        return min(sel_exchanges, key=lambda x: Maze.getManhattanDistance(self.current_cell, x), default=None)

    def getGoal(self):
        logging.info("State: " + str(self.state))
        # print("State: " + str(self.state))
        if self.state == 1:
            if not self.target_exchange:
                self.target_exchange = self.getBestRateExchange()

            if self.rupees == 0 and self.time < WANDER_TIME and self.isInRadius(self.target_exchange, EXCHANGE_RADIUS):
                self.state = 2

            if self.rupees > 0 or self.time > WANDER_TIME:
                self.state = 3

            return self.target_exchange

        elif self.state == 2:

            if self.rupees > 0 or self.time > WANDER_TIME:
                self.state = 3
            elif self.isAtCell(CENTER_CELL):
                self.target_exchange =  self.getBestRateExchange()
                self.state = 1
            return CENTER_CELL

        elif self.state == 3:
            if self.rupees >= 1500:
                self.target_exchange = self.getGoldenExchange()
                if self.target_exchange:
                    self.state = 4

            if self.rupees >= 5000 and self.exchanges_changed:
                self.target_exchange = self.getHighestValueExchange()
                if self.isInRadius(self.target_exchange, REACHABLE_RADIUS):
                    self.state = 4

            if self.target_drop is None or self.isAtCell(self.target_drop):
                sorted_drops = sorted(self.money_cells, key=lambda drop: Maze.getManhattanDistance(self.current_cell, drop))
                closest_drops = sorted_drops[0:3]
                self.target_drop = min(closest_drops, key=lambda drop: self.maze.getPathLength(self.current_cell, drop))

            really_close_drop = self.getReallyCloseDrop(self.money_cells)
            if really_close_drop:
                return really_close_drop
            return self.target_drop

        elif self.state == 4:
            self.target_drop = None

            if self.exchanges_changed:
                self.target_exchange = self.getHighestValueExchange()
                if not self.isInRadius(self.target_exchange, REACHABLE_RADIUS):
                    self.state = 3

            if self.rupees <= 1000 or self.isAtCell(self.target_exchange):
                self.state = 3

            if not self.isInRadius(self.target_exchange, REALLY_CLOSE_RADIUS):
                really_close_drop = self.getReallyCloseDrop(self.money_cells)
                if really_close_drop:
                    return really_close_drop
            return self.target_exchange
        