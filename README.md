# AMR-Sim
Simulated Hospital Disinfection Robot
# Project Overview
This repository is the simulation for an Autonomous Mobile Robot (AMR) designed for hospital floor sanitization. The project was developed for a Junior Design course to demonstrate efficient area coverage.
The robot utilizes a TurtleBot3 Burger platform equipped with an LDS-01 Lidar sensor to navigate hospital corridors and patient rooms autonomously.
# Logic
The primary challenge in this simulation was moving beyond simple reactive behavior (which causes high-frequency oscillations or "twitching") toward a more intentional navigation strategy. The robot implements a Lawnmower-like sweep. Unlike random-bounce algorithms, this structured approach ensures that every "tile" of the floor is systematically covered. The robot operates using a finite state machine (FSM) with timed transitions (Hysteresis). This ensures that once a maneuver (like a 90-degree turn) begins, it is completed before the sensors re-evaluate the environment, preventing logical deadlocks in tight corners.
# How to Run
1. Clone the repository to your local machine.
2. Open Webots and load the .wbt world file located in the worlds/ folder.
3. Set the controller of the TurtleBot3 Burger to burger_code (controllers/burger_code/burger_code.py).
4. Run the simulation.
