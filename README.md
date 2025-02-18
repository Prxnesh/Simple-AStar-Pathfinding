# A* Algorithm Implementation

## Overview
This project is an implementation of the **A* (A-star) search algorithm** in Python. The A* algorithm is a widely used pathfinding and graph traversal algorithm that efficiently finds the shortest path from a start node to a goal node.

This project is part of my **Artificial Intelligence course assignment** at **SRM University, Chennai**.

---

## About the A* Algorithm
A* is an informed search algorithm that uses:
1. **g(n)**: The cost from the start node to the current node.
2. **h(n)**: The heuristic estimate of the cost from the current node to the goal.
3. **f(n) = g(n) + h(n)**: The total estimated cost of the cheapest path through `n`.

A* is widely used in applications like:
- **Game Development** (Pathfinding for NPCs)
- **Robotics** (Navigation and obstacle avoidance)
- **Maps and GPS systems**
- **Network Routing**

---

## Example Grid Representation
```
S  #  .  .  .
.  #  .  #  .
.  .  .  #  .
.  #  #  #  .
.  .  .  .  G
```
- `S` - Start Position
- `G` - Goal Position
- `#` - Obstacles
- `.` - Open Path

---

## How to Modify the Grid
- Open the Python script and locate the `grid` variable.
- Modify the grid structure by changing `0` (open path) and `1` (obstacles) to create a custom layout.
- Update the `start` and `goal` positions to test different scenarios.

---

## How to Run the Code
1. Clone this repository:
   ```sh
   git clone https://github.com/yourusername/astar-search.git
   ```
2. Navigate to the project directory:
   ```sh
   cd astar-search
   ```
3. Run the script:
   ```sh
   python astar.py
   ```

---

## Future Enhancements
- Implement different heuristic functions (Euclidean, Diagonal distance, etc.)
- Add visualization for the search process
- Expand support for larger grid-based maps

---

## Conclusion
This implementation of the **A* algorithm** demonstrates its efficiency in solving pathfinding problems. It is widely used in AI and robotics due to its optimal and complete nature.

Feel free to contribute and enhance this project! 😊
