# A*-Algorithm-for-Motion-Planning
A brief implementation for 2D motion planning using A-star algorithm


## Overview
This project implements the A* (A-star) path finding algorithm using MATLAB. A* is a popular algorithm for finding the shortest path between two points on a grid, efficiently handling obstacles and different terrain costs. A* uses a best-first search approach to find the least-cost path from a given start node to a goal node. It uses a heuristic function to estimate the cost from any node to the goal, which helps guide the search more efficiently than algorithms like Dijkstra's.

Key components of our implementation:
- Heuristic Function: Manhattan distance
- Cost Function: Uniform cost for all traversable cells
- Neighbor Selection: 4-connectivity (up, down, left, right)


The main function `AStarGrid` takes an input map, start and goal coordinates, and returns the optimal path along with the number of nodes expanded during the search. To run the algorithmn ensure you have MATLAB installed. Then, place the `AStarGrid.m` file in your MATLAB path, and run the sample input file provided. You can see the output and plot in output file.
