<h2> A* Path Planning with Differential Drive Robot Simulation</h2>

This MATLAB project implements the A\* pathfinding algorithm to navigate a 30x30 grid map with obstacles. Additionally, a differential drive robot follows the computed path.

## Features

- **A*** Algorithm Implementation*\*: Finds the shortest path from a start to a goal location while avoiding obstacles.
- **Grid-Based Environment**: A 30x30 grid map with obstacles for realistic path planning.
- **Differential Drive Robot Simulation**: Simulates a robot moving along the computed path using realistic motion dynamics.
- **Visualization**: Displays the grid, obstacles, computed path, and robot movement.

## Installation

Ensure you have MATLAB installed on your system. No additional toolboxes are required.

## Usage

1. Clone this repository or download the script.
2. Open MATLAB and run the script.
3. The A\* algorithm computes the path, and the differential drive robot follows it.

## Code Overview

### Grid Map and Obstacles

- The environment is defined as a 30x30 grid.
- Obstacles are placed at specific locations.

### A\* Algorithm

- Utilizes an 8-direction movement model.
- Implements a priority queue using the `openList`.
- Uses the Euclidean distance heuristic.

### Differential Drive Simulation

- The robot follows the computed path using a proportional controller for angular velocity.
- The visualization updates in real-time as the robot moves.

## Example Output

After running the script, MATLAB displays:

- The grid map with obstacles.
- The computed A\* path in red.
- The robot moving along the path dynamically.

## Functions

### `AStar(grid, startNode, goalNode)`

Computes the optimal path using the A\* algorithm.

### `heuristic(node, goal)`

Calculates the Euclidean distance between a node and the goal.

### `reconstructPath(cameFrom, current)`

Reconstructs the path from the goal to the start.

### `DifferentialDriveSimulation(path)`

Simulates the motion of a differential drive robot following the computed path.

### `drawRobot(x, y, theta, length, width)`

Draws the robot at a given position and orientation.

### `updateRobot(robotBody, x, y, theta, length, width)`

Updates the robot's position and orientation during movement.

## Future Improvements

- Add support for real-world map integration.
- Implement dynamic obstacle avoidance.
- Enhance visualization with additional sensors.

##
