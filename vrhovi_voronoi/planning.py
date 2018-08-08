import networkx as nx
import numpy as np
from queue import PriorityQueue

def heuristic(p1,p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def create_graph(edges):
    G = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = heuristic(p1,p2)
        G.add_edge(p1, p2, weight = dist)
    return G

def a_star(graph, heuristic, start, goal):

    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    branch = {}
    found = False

    while not queue.empty():
        current_cost, current_node = queue.get()
        if current_node == goal:
            print("Found a path")
            found = True
            break
        else:
            for next_node in graph[current_node]:
                new_cost = current_cost + graph.edges[current_node, next_node]['weight'] + heuristic(next_node, goal)
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)

    path_cost = 0
    if found:
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    return path[::-1], path_cost


def closest_point(graph, current_node):
    closest_point = None
    dist = 10000000
    for p in graph.nodes:
        d = heuristic(p, current_node)
        if d < dist:
            closest_point = p
            dist = d
    return closest_point