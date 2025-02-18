import heapq
import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # Cost from start to current node
        self.h = h  # Heuristic (estimated cost to goal)
        self.f = g + h  # Total cost

    def __lt__(self, other):  # For priority queue
        return self.f < other.f

def heuristic(a, b):
    """Calculate Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal, visualize=True):
    """
    Find shortest path using A* with optional visualization.

    Parameters:
    grid (list of list of int): A 2D list representing the grid where 0 represents an open cell and 1 represents an obstacle.
    start (tuple of int): A tuple representing the starting position (x, y) in the grid.
    goal (tuple of int): A tuple representing the goal position (x, y) in the grid.
    visualize (bool, optional): A boolean indicating whether to visualize the exploration process. Default value is True.

    Returns:
    list of tuple or None: A list of tuples representing the shortest path from the start to the goal. If no path is found, it returns None.
    """
    open_list = []
    closed_set = set()
    start_node = Node(start, None, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    # Visualization setup
    if visualize:
        plt.figure(figsize=(6, 6))
        draw_grid(grid, start, goal, path=[], visited=[])
        plt.pause(1)

    visited_nodes = []

    while open_list:
        current = heapq.heappop(open_list)

        if current.position == goal:
            path = []
            while current:
                path.append(current.position)
                current = current.parent
            path.reverse()

            if visualize:
                draw_grid(grid, start, goal, path, visited_nodes)
                plt.show()

            return path  # Return the final path

        closed_set.add(current.position)
        visited_nodes.append(current.position)

        # Visualization of exploration
        if visualize:
            draw_grid(grid, start, goal, path=[], visited=visited_nodes)
            plt.pause(0.1)

        x, y = current.position

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Move up, down, left, right
            new_pos = (x + dx, y + dy)
            if new_pos in closed_set:
                continue

            if not (0 <= new_pos[0] < len(grid) and 0 <= new_pos[1] < len(grid[0])):  # Out of bounds
                continue

            if grid[new_pos[0]][new_pos[1]] == 1:  # Obstacle
                continue

            new_node = Node(new_pos, current, current.g + 1, heuristic(new_pos, goal))
            heapq.heappush(open_list, new_node)

    return None  # No path found


def draw_grid(grid, start, goal, path, visited):
    """Draw the grid, start, goal, visited nodes, and path."""
    grid_copy = np.array(grid)

    plt.clf()
    plt.imshow(grid_copy, cmap="gray_r")  # Display grid (0 = white, 1 = black)

    # Mark start and goal
    plt.scatter(start[1], start[0], c="green", marker="o", label="Start")  # Green for start
    plt.scatter(goal[1], goal[0], c="red", marker="o", label="Goal")  # Red for goal

    # Mark visited nodes
    for v in visited:
        plt.scatter(v[1], v[0], c="yellow", s=10, alpha=0.5)  # Yellow for explored nodes

    # Mark path
    if path:
        for p in path:
            plt.scatter(p[1], p[0], c="blue", s=50)  # Blue for final path

    plt.legend()
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.grid(True)
    plt.pause(0.1)

# Example Grid (0 = open, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]

start = (0, 0)  # Start position
goal = (4, 4)   # Goal position

path = astar(grid, start, goal, visualize=True)

if path:
    print("Shortest Path:", path)
else:
    print("No path found!")
