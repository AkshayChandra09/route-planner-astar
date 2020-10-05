import math
from queue import PriorityQueue
from MapsData import Map_10, Map_40


def get_path(prev, start, goal):
    current_node = goal
    path = [current_node]
    while current_node != start:
        current_node = prev[current_node]
        path.append(current_node)
    # reverse array because it is stored from goal to start
    path.reverse()
    return path


def get_distance(from_node, to_node):
    # math.dist is new in Python 3.8
    # eu_distance = math.dist(from_node, to_node)
    # return eu_distance
    a = (from_node[0] - to_node[0]) ** 2
    b = (from_node[1] - to_node[1]) ** 2
    return math.sqrt(a + b)


def shortest_path(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    parent = {start: None}
    explored = {start: 0}
    while not frontier.empty():
        current_node = frontier.get()
        if current_node == goal:
            get_path(parent, start, goal)
        for node in graph.roads[current_node]:
            f_value = explored[current_node] + get_distance(graph.intersections[current_node], graph.intersections[node])
            if node not in explored or f_value < explored[node]:
                explored[node] = f_value
                total_f_value = f_value + get_distance(graph.intersections[current_node], graph.intersections[node])
                frontier.put(node, total_f_value)
                parent[node] = current_node
    return get_path(parent, start, goal)


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
