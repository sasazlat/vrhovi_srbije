import numpy as np
from grid import create_grid_and_edges
import matplotlib.pyplot as plt 
import pickle
import networkx as nx
from planning import heuristic, a_star, closest_point, create_graph

#data = np.loadtxt('colliders.csv', delimiter=',',dtype='Float64', skiprows=2)
data = np.loadtxt('vrhovi_new.csv', delimiter=',',dtype='Float64', skiprows=2)


print(data)

new_data = []
for row in data:
    if np.absolute(row[0]) > 4000 or np.absolute(row[1]) > 2000:
        continue
    else:
        new_data.append(row)

small_data = np.array(new_data) / 10
print(small_data)

#s_data = np.array(data)/10
grid, edges = create_grid_and_edges(small_data, 1, 0.5)

grid_pickle_out = open("grid_p.pickel", mode='wb')
pickle.dump(grid, grid_pickle_out)
grid_pickle_out.close()


outfile = open("edges_p.pickel", mode='wb')
pickle.dump(edges, outfile)
outfile.close()

in_grid = open("grid_p.pickel", mode='rb')
grid = pickle.load(in_grid)
in_grid.close()

in_edges = open("edges_p.pickel", mode='rb')
edges = pickle.load(in_edges)
in_edges.close()

print('Fount %5d edges' % len(edges))


plt.imshow(grid, origin='lower', cmap='Greys')
#plt.show()
for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


G = create_graph(edges)

start = 34
goal = 300

start_closest = closest_point(G, start)
goal_closest = closest_point(G,goal)

path, cost = a_star(G, heuristic, start_closest, goal_closest)
print (len(path))

