import numpy as np
import cv2
import time
from queue import PriorityQueue

class Obstacle:
    """
    Represents an obstacle in the grid.
    """
    def __init__(self, shape, params):
        self.shape = shape
        self.params = params

    def contains(self, x, y):
        """
        Check if a point (x, y) is inside the obstacle.
        """
        if self.shape == 'rectangle':
            x1, y1, x2, y2 = self.params
            return x1 <= x <= x2 and y1 <= y <= y2
        elif self.shape == 'hexagon':
            x_center, y_center, side = self.params
            return np.abs(x - x_center) / side + np.abs(y - y_center) / (side * np.sqrt(3)/2) <= 1
        elif self.shape == 'custom_shape':
            return self.point_in_polygon(x, y)
        else:
            return False
        
    def point_in_polygon(self, x, y):
        polygon = self.params
        num = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(num + 1):
            p2x, p2y = polygon[i % num]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

class Grid:
    """
    Manages the grid, obstacles, and pathfinding.
    """
    def __init__(self, width, height, obstacles=[]):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.clearance = 5
        self.grid = self.generate_grid()

    def generate_grid(self):
        """
        Generates a grid with obstacles and clearance.
        """
        grid = np.full((self.height, self.width, 3), 255, dtype=np.uint8)  # White background

        for y in range(self.height):
            for x in range(self.width):
                for obstacle in self.obstacles:
                    if obstacle.contains(x, y):
                        grid[y, x] = [0, 0, 0]  # Black for obstacles
                        break
                    # Add clearance
                    elif any(obstacle.contains(x + dx, y + dy) for dx in range(-self.clearance, self.clearance+1) for dy in range(-self.clearance, self.clearance+1)):
                        grid[y, x] = [192, 192, 192]  # Gray for clearance
        return grid

    def is_valid_node(self, node):
        """
        Check if the node is valid (not an obstacle or outside the grid).
        """
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height and (self.grid[y, x] == [255, 255, 255]).all()

    def visualize(self, path=None):
        """
        Visualize the grid, obstacles, clearance, and optionally a path.
        """
        img = self.grid.copy()
        if path:
            for x, y in path:
                
                img[y, x] = [0, 0, 255]  # Red for path

        cv2.imshow("Pathfinding Visualization", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def dijkstra(grid, start_node, goal_node):
    """
    Dijkstra's algorithm for finding the shortest path from start_node to goal_node.
    """
    open_list = PriorityQueue()
    open_list.put((0, start_node))
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}

    while not open_list.empty():
        current_cost, current_node = open_list.get()

        if current_node == goal_node:
            break

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # 8 directions
            next_node = (current_node[0] + dx, current_node[1] + dy)
            new_cost = current_cost + (np.sqrt(2) if dx != 0 and dy != 0 else 1)
            if grid.is_valid_node(next_node) and (next_node not in cost_so_far or new_cost < cost_so_far[next_node]):
                cost_so_far[next_node] = new_cost
                priority = new_cost
                open_list.put((priority, next_node))
                came_from[next_node] = current_node

    # Reconstruct path
    path = []
    current_node = goal_node
    while current_node != start_node:
        path.append(current_node)
        current_node = came_from[current_node]
    path.append(start_node)
    path.reverse()
    return path

# Example usage

e_shape_vertices = [(900, 450), (1100, 450), (1100, 50), (900, 50), (900, 125),
                    (1020, 125), (1020, 375), (900, 375)]  # Define "E" shape vertices
obstacles = [
    Obstacle('rectangle', (100, 0, 175, 400)),  # Parameters: x1, y1, x2, y2
    Obstacle('rectangle', (275, 100, 350, 500)),    # Parameters: x1, y1, x2, y2
    Obstacle('hexagon', (650, 250, 150)),       # Parameters: x_center, y_center, side
    Obstacle('custom_shape', e_shape_vertices)  # Adding custom "E" shaped obstacle
]
grid = Grid(1200, 500, obstacles)
x1 = int(input("Please enter the x coordinate of the start node of the point robot, x1: "))
y1 = int(input("Please enter the y coordinate of the start node of the point robot, y1: "))
x2 = int(input("Please enter the x coordinate of the goal node of the point robot, x2: "))
y2 = int(input("Please enter the y coordinate of the goal node of the point robot, y2: "))
y1_transformed = 500 - y1
y2_transformed = 500 - y2

start_node = (x1, y1_transformed)   # Transforming the input coordinates
# To take coordinates from the user in a coordinate system where the origin lies in the bottom left corner.  
goal_node = (x2, y2_transformed)

start_time = time.time()
path = dijkstra(grid, start_node, goal_node)
end_time = time.time()

if path:
    print("Goal Node Achieved")
    print("Path:", path)
    print("Time Taken:", end_time - start_time, "seconds")
    grid.visualize(path)
else:
    print("Path Not Found")
