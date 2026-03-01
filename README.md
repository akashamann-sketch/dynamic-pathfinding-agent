# dynamic-pathfinding-agent
Dynamic Pathfinding Agent using A* and GBFS (Matplotlib GUI)
Dynamic Pathfinding Agent

This project implements a Dynamic Pathfinding Agent using:
A* Search Algorithm
Greedy Best First Search (GBFS)
The agent navigates a grid environment with obstacles and visualizes the search process using Matplotlib.
The visualization shows:
Frontier Nodes (Yellow)
Visited Nodes (Blue)
Final Path (Green)
Obstacles (Black)

Install Matplotlib using:
pip install matplotlib

How to Run:
Open a terminal in the project folder and run:
python dynamic_pathfinder.py
If Python does not work, try:
py dynamic_pathfinder.py

Controls:
Inside the program window:
Mouse Click
Add or remove obstacles
A Key:
Run A* Search
G Key:
Run Greedy Best First Search
R Key:
Reset the grid

Algorithms Used
A* Search
A* search uses:
f(n) = g(n) + h(n)
Where:
g(n) = distance from start
h(n) = heuristic distance to goal
A* produces the shortest path in most cases.

Greedy Best First Search (GBFS)
GBFS uses:
f(n) = h(n)
It selects nodes based only on heuristic distance.
GBFS is usually faster but does not always produce the shortest path.
