import numpy as np
import matplotlib.pyplot as plt
from heapq import heappop, heappush

class PathPlanner:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.grid = np.zeros(grid_size, dtype=int)  # 0: free, 1: obstacle
        self.path = []

    def mark_obstacle(self, path):
        # Mark the path as obstacles in the grid
        for x, y in path:
            self.grid[y, x] = 1

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        open_set = []
        heappush(open_set, (0 + self.heuristic(start, goal), 0, start, []))

        visited = set()

        while open_set:
            _, cost, current, path = heappop(open_set)

            if current in visited:
                continue

            visited.add(current)
            path = path + [current]

            if current == goal:
                return path

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < self.grid_size[1] and
                        0 <= neighbor[1] < self.grid_size[0] and
                        neighbor not in visited and
                        self.grid[neighbor[1], neighbor[0]] == 0):
                    heappush(open_set, (cost + 1 + self.heuristic(neighbor, goal), cost + 1, neighbor, path))

        return None

    def plot_grid(self):
        plt.imshow(self.grid, cmap="gray_r")
        plt.xticks(np.arange(self.grid_size[1]), labels=np.arange(self.grid_size[1]))
        plt.yticks(np.arange(self.grid_size[0]), labels=np.arange(self.grid_size[0]))
        plt.grid(color="black", linestyle="-", linewidth=0.5)

    def plot_path(self, path, goal):
        self.plot_grid()
        x, y = zip(*path)
        plt.plot(x, y, marker="o", color="blue")
        
        # Highlight the destination point in red
        goal_x, goal_y = goal
        plt.plot(goal_x, goal_y, marker="o", color="red", markersize=8)

        plt.pause(0.1)  # Non-blocking pause for updating the plot

if __name__ == "__main__":
    planner = PathPlanner((20, 20))

    # Define 3 points the robot needs to reach
    points = [(1, 1), (18, 18), (10, 5)]  
    previous_path = []

    # Start the robot moving through each of the points
    for i in range(1, len(points)):
        start = points[i-1]
        goal = points[i]

        # Plot grid before each path calculation
        print(f"Calculating path from {start} to {goal}...")
        path = planner.a_star(start, goal)
        
        if path is None:
            print(f"No valid path found from {start} to {goal}!")
            break

        print(f"Found path from {start} to {goal}: {path}")

        # Mark the path as obstacles so the robot avoids it in the next segment
        planner.mark_obstacle(path)  # Mark the path as an obstacle

        # Visualize the path and grid with obstacles marked, and highlight the destination in red
        planner.plot_path(path, goal)  # Plot the path and highlight the destination

        previous_path += path  # Add the current path to the previous paths

    print("Final destination reached.")
    plt.show()  # Final display of the grid after all paths are calculated
