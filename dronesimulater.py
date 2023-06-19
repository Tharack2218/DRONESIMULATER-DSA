import random
import time
from collections import deque
import heapq


class DroneSimulator:
    def __init__(self, grid_size, start_index, target_index, obstacles):
        self.grid_size = grid_size
        self.drone_position = start_index
        self.target_position = target_index
        self.obstacles = obstacles
        self.path = self.calculate_shortest_path()
        self.speed = 0
        self.total_time = 0
        self.altitude_variations = []
        self.topology = []
        self.timestamp_queue = deque()
        self.stop_queue = []
        self.spatial_index = self.build_spatial_index()
        self.camera_images = {}
        self.distance_traveled = 0
        self.battery_level = 100
        self.path_graph = None
        self.topology_tree = None

    def build_spatial_index(self):
        spatial_index = {}
        for obstacle in self.obstacles:
            x, y, z = obstacle
            if x not in spatial_index:
                spatial_index[x] = {}
            if y not in spatial_index[x]:
                spatial_index[x][y] = set()
            spatial_index[x][y].add(z)
        return spatial_index

    def calculate_shortest_path(self):
        distances = {self.drone_position: 0}
        queue = [(0, self.drone_position)]
        previous = {}
        while queue:
            current_distance, current_position = heapq.heappop(queue)
            if current_position == self.target_position:
                path = []
                while current_position in previous:
                    path.append(current_position)
                    current_position = previous[current_position]
                return path[::-1]
            if (
                    current_position in distances
                    and current_distance > distances[current_position]
            ):
                continue
            distances[current_position] = current_distance
            neighbors = self.get_neighbors(current_position)
            for neighbor in neighbors:
                if neighbor not in self.obstacles:
                    new_distance = (
                            current_distance + self.get_distance(current_position, neighbor)
                    )
                    if (
                            neighbor not in distances
                            or new_distance < distances[neighbor]
                    ):
                        heapq.heappush(queue, (new_distance, neighbor))
                        previous[neighbor] = current_position

    def get_neighbors(self, position):
        x, y, z = position
        neighbors = []
        if x > 0:
            neighbors.append((x - 1, y, z))
        if x < self.grid_size - 1:
            neighbors.append((x + 1, y, z))
        if y > 0:
            neighbors.append((x, y - 1, z))
        if y < self.grid_size - 1:
            neighbors.append((x, y + 1, z))
        if z > 0:
            neighbors.append((x, y, z - 1))
        if z < self.grid_size - 1:
            neighbors.append((x, y, z + 1))
        return neighbors

    def get_distance(self, position1, position2):
        x1, y1, z1 = position1
        x2, y2, z2 = position2
        return abs(x2 - x1) + abs(y2 - y1) + abs(z2 - z1)

    def move_drone(self):
        if self.path:
            self.drone_position = self.path.pop(0)
            self.distance_traveled += 1
            return self.drone_position
        return None

    def simulate(self):
        start_time = time.time()
        while self.move_drone():
            self.analyze_path()
            if self.check_obstacle_collision():
                print("Obstacle collision detected!")
                break
            self.decrease_battery_level()
            self.print_grid()  # Display grid after each movement
            time.sleep(1)  # Delay between movements (1 second)
        end_time = time.time()
        self.total_time = end_time - start_time
        if self.total_time > 0:
            self.speed = self.calculate_speed()
        self.build_path_graph()
        self.build_topology_tree()

    def analyze_path(self):
        altitude = self.drone_position[2]
        self.altitude_variations.append(altitude)
        self.topology.append(self.drone_position)
        timestamp = time.time()
        self.timestamp_queue.append((self.drone_position, timestamp))

    def calculate_speed(self):
        if len(self.timestamp_queue) > 1:
            position1, timestamp1 = self.timestamp_queue[0]
            position2, timestamp2 = self.timestamp_queue[-1]
            distance = self.get_distance(position1, position2)
            time_difference = timestamp2 - timestamp1
            speed = distance / time_difference
            return speed
        return 0

    def check_obstacle_collision(self):
        x, y, z = self.drone_position
        if (
                self.spatial_index.get(x, {}).get(y, set()) == {z}
                or self.spatial_index.get(x, {}).get(y, set()) == {z - 1}
                or self.spatial_index.get(x, {}).get(y, set()) == {z + 1}
        ):
            return True
        return False

    def print_distance_traveled(self):
        print(f"Distance Traveled: {self.distance_traveled} units")

    def decrease_battery_level(self):
        self.battery_level -= 1

    def print_battery_level(self):
        print(f"Battery Level: {self.battery_level}%")

    def print_grid(self):
        print("\nGrid:")
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                for k in range(self.grid_size):
                    position = (i, j, k)
                    if position == self.drone_position:
                        print("D", end=" ")
                    elif position == self.target_position:
                        print("T", end=" ")
                    elif position in self.obstacles:
                        print("#", end=" ")
                    else:
                        print(".", end=" ")
                print()
            print()

    def print_time_taken(self):
        print(f"Time taken: {self.total_time:.2f} seconds")

    def print_speed(self):
        print(f"Drone speed: {self.speed:.2f} units/second")

    def print_altitude_variations(self):
        print("\nAltitude Variations:")
        print(self.altitude_variations)

    def print_topology(self):
        print("\nPath Topology:")
        print(self.topology)

    def build_path_graph(self):
        self.path_graph = {}
        for i in range(len(self.topology) - 1):
            current_node = self.topology[i]
            next_node = self.topology[i + 1]
            if current_node not in self.path_graph:
                self.path_graph[current_node] = set()
            self.path_graph[current_node].add(next_node)

    def build_topology_tree(self):
        self.topology_tree = {}
        root = self.topology[0]
        self.build_topology_tree_helper(root)

    def build_topology_tree_helper(self, node):
        if node not in self.path_graph:
            return
        children = self.path_graph[node]
        for child in children:
            if child not in self.topology_tree:
                self.topology_tree[child] = []
            self.topology_tree[child].append(node)
            self.build_topology_tree_helper(child)

    def print_path_graph(self):
        print("\nPath Graph:")
        for node, neighbors in self.path_graph.items():
            print(f"Node: {node}")
            print(f"Neighbors: {neighbors}")
            print()

    def print_topology_tree(self):
        print("\nTopology Tree:")
        for node, parents in self.topology_tree.items():
            print(f"Node: {node}")
            print(f"Previous Destination: {parents}")
            print()


# User input prompts
grid_size = int(input("Enter the size of the grid: "))
start_index = tuple(map(int, input("Enter the starting position (x, y, z): ").split(",")))
target_index = tuple(map(int, input("Enter the target position (x, y, z): ").split(",")))
num_obstacles = int(input("Enter the number of obstacles: "))
obstacles = set()
for _ in range(num_obstacles):
    obstacle = tuple(map(int, input("Enter the obstacle position (x, y, z): ").split(",")))
    obstacles.add(obstacle)

# Create drone simulator instance
drone_simulator = DroneSimulator(grid_size, start_index, target_index, obstacles)

# Run simulation
drone_simulator.simulate()

drone_simulator.print_distance_traveled()
drone_simulator.print_battery_level()
drone_simulator.print_time_taken()
drone_simulator.print_speed()
drone_simulator.print_altitude_variations()
drone_simulator.print_topology()
drone_simulator.print_path_graph()
drone_simulator.print_topology_tree()