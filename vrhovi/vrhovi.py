import pandas as pd
import numpy as np
import csv
import re
import  matplotlib.pyplot as plt 
from skimage.morphology import medial_axis
from skimage.util import invert
from grid import create_grid
from planning import a_star
from bresenham import bresenham
import utm
import pickle

vrhovi_fajl = "vrhovi__srbije-Fractal-unicode.csv"
geodetic_home_coordinates = [21.931919, 43.333313, 200] #long, lat, alt
geodetic_arp_coordinates = [21.853722, 43.337289, 198]
data_regex = re.compile(r'[-+]?\d*\.\,\d+|\d+')

def global_to_local_tops(global_position, global_home):
    
    # TODO: Get easting and northing of global_home
    (long_home, lat_home, alt_home) = global_home
    (easting_home, northing_home, zone_home, zone_home_letter) = utm.from_latlon(lat_home, long_home)
    # TODO: Get easting and northing of global_position
    (long_global, lat_global, alt_global) = global_position
    (easting_global, northing_global, zone_global, zone_global_letter) = utm.from_latlon(lat_global, long_global)
    # TODO: Create local_position from global and home positions
    local_position = np.array([northing_global - northing_home, easting_global - easting_home, alt_global - alt_home])
    
    return local_position

def tops(csv_fajl):
    vrhovi_list = []
    with open(csv_fajl, newline='', encoding='utf-8') as f:
        next(f)
        csv_reader = csv.reader(f)
        for row in csv_reader:
            rb, vrh, planina, alt, lat, lon, sekcija, napomena = row[:]
            reg_N = re.findall(data_regex, lat)
            reg_E = re.findall(data_regex, lon)
            coord_N = 0
            coord_E = 0

            if reg_N:
                deg, min, sec = reg_N[:]
                m = int(min) + int(sec) / 1000
                coord_N = int(deg) + m / 60
            if reg_E:
                deg, min, sec = reg_E[:]
                m = int(min) + int(sec) / 1000
                coord_E = int(deg) + m / 60
            N,E,D = global_to_local_tops([coord_E, coord_N, int(alt)], geodetic_arp_coordinates)
            #new = np.append([N/1000,E/1000,D, 600, 600, D])
            vrhovi_list.append([N,E,D, 600, 600, D])
    return vrhovi_list

#vrhovi_np = np.array(tops(vrhovi_fajl)) / 1000

#print(vrhovi_np)
#np.savetxt('vrhovi_new.csv', vrhovi_np, delimiter=',')
#data = np.loadtxt("vrhovi_new.csv", delimiter=',', dtype='Float64')

#print(data)

#new_data = []
#for row in data:
#    if np.absolute(row[0]) > 300 or np.absolute(row[1]) > 300:
#        continue
#    else:
#        new_data.append(row)

#small_data = np.array(new_data)
#print(small_data)

start_ne = (44,  87)
goal_ne = (150, 250)

drone_altitude = 0.055
safety_distance = 0.2


#grid = create_grid(small_data, drone_altitude, safety_distance)
#grid_pickle_out = open("grid_arp_pickel.pickel", mode='wb')
#pickle.dump(grid, grid_pickle_out)
#grid_pickle_out.close()


#skeleton = medial_axis(invert(grid))
#outfile = open("skel_arp_pickel.pickel", mode='wb')
#pickle.dump(skeleton, outfile)
#outfile.close()

grid_pickle_in = open("grid_arp_pickel.pickel", mode='rb')
grid_pickle = pickle.load(grid_pickle_in)
grid_pickle_in.close()

skel_pickle_in = open("skel_arp_pickel.pickel", mode='rb')
skel_pickle = pickle.load(skel_pickle_in)
skel_pickle_in.close()

plt.imshow(grid_pickle, cmap='Greys', origin='lower')
plt.imshow(skel_pickle, cmap='Greys', origin='lower', alpha=0.7)
    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


def find_start_goal(skel, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    non_zero = np.nonzero(skel)
    skel_cells = np.transpose(non_zero)
    dista = np.linalg.norm(np.subtract(start, skel_cells), axis=1).argmin()
    near_start = skel_cells[dista]
    near_goal = skel_cells[np.linalg.norm(np.subtract(goal, skel_cells), axis=1).argmin()]
    return near_start, near_goal


skel_start, skel_goal = find_start_goal(skel_pickle, start_ne, goal_ne)

print(start_ne, goal_ne)
print(skel_start, skel_goal)


# In[9]:
def heuristic_func(position, goal_position):
    return np.linalg.norm(np.subtract(position, goal_position))



path, cost = a_star(invert(skel_pickle).astype(np.int), heuristic_func, tuple(skel_start), tuple(skel_goal))
print("Path length = {0}, path cost = {1}".format(len(path), cost))

# Compare to regular A* on the grid
#path2, cost2 = a_star(grid_pickle, heuristic_func, start_ne, goal_ne)
#print("Path length = {0}, path cost = {1}".format(len(path2), cost2))


# In[14]:
plt.imshow(grid_pickle, origin='lower')
plt.imshow(skel_pickle, cmap='Greys', origin='lower', alpha=0.7)
    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.plot(skel_start[1], skel_start[0], 'gx')
plt.plot(skel_goal[1], skel_goal[0], 'gx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
#plt.show()
curr_pos = skel_start
actual_path = []
actual_path2 = []
for action in path:
    curr_pos = (curr_pos[0] + action.delta[0], curr_pos[1] + action.delta[1])
    #curr_pos = (curr_pos[0] + action.delta[0], curr_pos[1] + action.delta[1])
    actual_path.append(curr_pos)
    actual_path2.append(curr_pos)

def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1,p3 = pruned_path[i], pruned_path[i + 2]
        br = list(bresenham(p1[0], p1[1], p3[0], p3[1]))
        if all((grid_pickle[p] == 0) for p in br):
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path
pruned_path = prune_path(actual_path)
pp = np.array(pruned_path)
pp2 = np.array(actual_path2)

#pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
#pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'r')
plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()