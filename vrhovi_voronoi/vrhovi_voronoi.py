import numpy as np
from grid import create_grid_edges
import pickle

#data = np.loadtxt('vrhovi_new.csv', delimiter=',',dtype='Float64')


#print(data)

#new_data = []
#for row in data:
#    if np.absolute(row[0]) > 1000 or np.absolute(row[1]) > 1000:
#        continue
#    else:
#        new_data.append(row)

#small_data = np.array(new_data)
#print(small_data)

#grid, edges = create_grid_edges(small_data, 10, 10)

#grid_pickle_out = open("grid_p.pickel", mode='wb')
#pickle.dump(grid, grid_pickle_out)
#grid_pickle_out.close()


#outfile = open("edges_p.pickel", mode='wb')
#pickle.dump(edges, outfile)
#outfile.close()

in_grid = open("grid_p.pickel", mode='rb')
grid = pickle.load(in_grid)
in_grid.close()

in_edges = open("edges_p.pickel", mode='rb')
edges = pickle.load(in_edges)
in_edges.close()

print ('Fount %5d edges' % len(edges))


