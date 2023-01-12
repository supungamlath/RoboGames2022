# Dave Controller

This is the controller code for an E-puck robot simulated on Webots 2022 simulator.
The goal of the challenge is to collect as many rupees as possible in the shortest time and convert them into dollars by reaching designated goals in an unknown maze.

## Requirements

To run this code, you will need the following:

Python 3.9.5
The following Python libraries:
[OpenCV](https://opencv.org/)
[Scikit-learn](https://scikit-learn.org/)
[Scikit-multilearn](http://scikit.ml/)

## Installation

To install the required libraries, run the following command:

```
pip install -r requirements.txt
```

## Usage

To run Dave's controller, open the "FinalizedMaze.wbt" file in Webots 2022 and select the "Dave" robot.

Then, select the "Controller" tab and choose the "dave_controller.py" file as the controller.

## Code Structure

The code is structured as follows:

`dave_controller.py` is the main controller file, which contains the code that Dave will use to navigate the maze, collect rupees and convert them to dollars.

`maze.py` contains the code that is used to create the maze and the keep track of walls, visited cells and unknown cells.

`slam.py` contains the code that is used to perform SLAM (Simultaneous Localization and Mapping) using distance sensors and camera data.

`model.py` contains the code that is used to train a multi-label classification model that can predict the blocked cells from distance sensor readings.

`color_detector.py` contains the code that is used to detect objects from camera images.

`game.py` contains the logic for selecting exchanges and money drops in the optimal order.

## Approach

To solve the challenge, Dave's controller uses the following approach:

Receive data from the supervisor robot's emitter, including the location and exchange rate of the goals, the location of rupees, and Dave's current rupee and dollar balances.

Determine the goal with the highest exchange rate and plan a path to reach it.

Collect rupees along the way, making sure to keep track of Dave's rupee balance and pay taxes as necessary.

When Dave reaches the goal, convert all of his rupees into dollars at the current exchange rate.

Repeat the process until indefinitely.

## License

This code is licensed under the MIT License.
