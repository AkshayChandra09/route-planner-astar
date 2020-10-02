import math
import heapq
from MapsData import Map_10, Map_40
from queue import PriorityQueue


class Node:
    def __init__(self, node_number, coordinates, parent=0):
        self.node_number = node_number
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.parent = parent
        self.f_value = 0
        self.g_value = None
        self.h_value = None

    def get_location(self):
        return self.x, self.y

    def __eq__(self, other):
        return self.node_number == other.node_number

    def __ne__(self, other):
        return self.node_number != other.node_number

    def __lt__(self, other):
        return self.f_value < other.f_value


def get_distance(from_node, to_node):
    # math.dist is new in Python 3.8
    # eu_distance = math.dist(from_node.get_location(), to_node.get_location())
    # return eu_distance
    a = (from_node.x - to_node.x) ** 2
    b = (from_node.y - to_node.y) ** 2
    return math.sqrt(a + b)


def is_visited(value, visited_nodes):
    for node in visited_nodes:
        if node.node_number == value:
            return True
    return False


def shortest_path(map, start, goal):
    frontier = []
    explored = []
    start_node = Node(start, map.intersections[start])
    goal_node = Node(goal, map.intersections[goal])
    heapq.heappush(frontier, (0, start_node))

    while len(frontier) > 0:
        current_node = heapq.heappop(frontier)[1]
        explored.append(current_node)
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.node_number)
                current_node = current_node.parent
            path.append(start_node.node_number)
            # Return reversed path
            return path[::-1]

        for neighbor in map.roads[current_node.node_number]:
            if is_visited(neighbor, explored):
                continue
            node = Node(neighbor, map.intersections[neighbor], current_node)
            node.g_value = get_distance(current_node, node)  # path cost
            node.h_value = get_distance(node, goal_node)  # estimated distance
            node.f_value = current_node.f_value + node.g_value + node.h_value  # F value
            # heapq.heappush(frontier, (node.f_value, node))
            if add_to_nodes_list(frontier, node):
                heapq.heappush(frontier, (node.f_value, node))
    return None


def add_to_nodes_list(nodes_list, neighbor):
    for node in nodes_list:
        if neighbor == node[1] and neighbor.f_value >= node[1].f_value:
            return False
    return True


map = Map_40()
# test case 1
sp = shortest_path(map, 5, 34)
print("Shortest path is: {}".format(sp))

# test case 2
sp = shortest_path(map, 5, 25)
print("Shortest path is: {}".format(sp))

# test case 3
sp = shortest_path(map, 5, 28)
print("Shortest path is: {}".format(sp))

# test case 4
sp = shortest_path(map, 8, 24)
print("Shortest path is: {}".format(sp))




