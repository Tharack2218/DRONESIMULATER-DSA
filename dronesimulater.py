import time
import heapq
from collections import defaultdict

class Node:
    def __init__(self, position):
        self.position = position
        self.distance = float('inf')
        self.previous = None

    def __lt__(self, other):
        return self.distance < other.distance

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = defaultdict(list)

    def add_node(self, position):
        node = Node(position)
        self.nodes.append(node)

    def add_edge(self, node1, node2):
        self.edges[node1].append(node2)
        self.edges[node2].append(node1)

def dijkstra(graph, start_node):
    start_node.distance = 0
    queue = [(0, start_node)]

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_distance > current_node.distance:
            continue

        for neighbor in graph.edges[current_node]:
            distance = current_distance + 1  # Distance is always 1 unit
            if distance < neighbor.distance:
                neighbor.distance = distance
                neighbor.previous = current_node
                heapq.heappush(queue, (distance, neighbor))

def get_shortest_path(destination_node):
    path = []
    current_node = destination_node

    while current_node:
        path.insert(0, current_node.position)
        current_node = current_node.previous

    return path

class DroneSimulator:
    def __init__(self, grid_size, start_index, target_index, obstacles):
        self.grid_size = grid_size
        self.start_position = start_index
        self.target_position = target_index
        self.obstacles = obstacles
        self.path = []
        self.distance_traveled = 0
        self.speed = 0
        self.total_time = 0
        self.altitude_variations = []
        self.path_graph = None
        self.battery_level = 100

    def build_graph(self):
        graph = Graph()

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                for k in range(self.grid_size):
                    position = (i, j, k)
                    if position in self.obstacles:
                        continue
                    graph.add_node(position)

        for node1 in graph.nodes:
            for node2 in graph.nodes:
                if node1 != node2:
                    if abs(node1.position[0] - node2.position[0]) + \
                       abs(node1.position[1] - node2.position[1]) + \
                       abs(node1.position[2] - node2.position[2]) == 1:
                        graph.add_edge(node1, node2)

        return graph

    def simulate(self):
        graph = self.build_graph()

        start_node = None
        destination_node = None

        for node in graph.nodes:
            if node.position == self.start_position:
                start_node = node
            elif node.position == self.target_position:
                destination_node = node

        if start_node is None or destination_node is None:
            print("Invalid start or target position.")
            return

        dijkstra(graph, start_node)
        self.path = get_shortest_path(destination_node)

        start_time = time.time()

        for position in self.path:
            self.distance_traveled += 1
            self.analyze_path(position)

            if self.check_obstacle_collision():
                print("Obstacle collision detected!")
                break

            self.decrease_battery_level()
            self.print_grid(position)
            time.sleep(1)

        end_time = time.time()
        self.total_time = end_time - start_time

        if self.total_time > 0:
            self.speed = self.calculate_speed()

    def analyze_path(self, position):
        self.altitude_variations.append(position[2])

    def check_obstacle_collision(self):
        x, y, z = self.path[-1]
        if (x, y, z) in self.obstacles:
            return True
        return False

    def decrease_battery_level(self):
        self.battery_level -= 2

    def calculate_speed(self):
        if self.total_time > 0:
            return self.distance_traveled / self.total_time
        return 0

    def print_grid(self, current_position):
        print("\nGrid:")
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                for k in range(self.grid_size):
                    position = (i, j, k)
                    if position == current_position:
                        print("D", end=" ")
                    elif position == self.target_position:
                        print("T", end=" ")
                    elif position in self.obstacles:
                        print("#", end=" ")
                    else:
                        print(".", end=" ")
                print()
            print()

    def print_distance_traveled(self):
        print(f"Distance Traveled: {self.distance_traveled-1} units")

    def print_battery_level(self):
        print(f"Battery Level: {self.battery_level+2}%")

    def print_time_taken(self):
        print(f"Time taken: {self.total_time:.2f} seconds")

    def print_speed(self):
        print(f"Drone speed: {self.speed:.2f} units/second")

    def print_altitude_variations(self):
        print("\nAltitude Variations:")
        print(self.altitude_variations)

# Test the DroneSimulator
grid_size = int(input("Enter the size of the grid: "))
start_index = tuple(map(int, input("Enter the starting position (x, y, z): ").split(",")))
target_index = tuple(map(int, input("Enter the target position (x, y, z): ").split(",")))
num_obstacles = int(input("Enter the number of obstacles: "))
obstacles = set()
for _ in range(num_obstacles):
    obstacle = tuple(map(int, input("Enter the obstacle position (x, y, z): ").split(",")))
    obstacles.add(obstacle)

drone_simulator = DroneSimulator(grid_size, start_index, target_index, obstacles)
drone_simulator.simulate()

drone_simulator.print_distance_traveled()
drone_simulator.print_battery_level()
drone_simulator.print_time_taken()
drone_simulator.print_speed()
drone_simulator.print_altitude_variations()
