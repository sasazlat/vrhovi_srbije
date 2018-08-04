import numpy as np

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros([north_size, east_size])

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            n_min = int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1))
            n_max = int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1))
            e_min = int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1))
            e_max = int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1))
            grid[n_min:n_max+1, e_min:e_max+1] = 1

    return grid