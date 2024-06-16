from matplotlib import pyplot as plt
import numpy as np
from planning_utils import a_star, create_grid
from pruning_test import prune


def draw_grid(grid, edges, start_ne, goal_ne, path, pruned_path=None):
    # plt.imshow(np.flip(grid, 0))
    plt.imshow(grid, origin='lower', cmap='Greys') 

    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'rx')


    # Plotting the path
    if path is not None:
        for i in range(len(path)-1):
            plt.plot([path[i][1], path[i+1][1]], [path[i][0], path[i+1][0]], 'g-')  # Path in green
    # Plotting the pruned path
    if pruned_path is not None:
        for i in range(len(pruned_path)-1):
            plt.plot([pruned_path[i][1], pruned_path[i+1][1]], [pruned_path[i][0], pruned_path[i+1][0]], 'r-')  # Path in green


    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

local_position = (316, 444)
data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)

# Define a grid for a particular altitude and safety margin 5 obstacles
grid, north_offset, east_offset = create_grid(data, 5, 5)
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
# Define starting point on the grid (this is just grid center)
grid_start = (int(local_position[0] - north_offset), int(local_position[1] - east_offset))
# TODO: convert start position to current position rather than map center

# Set goal as some arbitrary position on the grid that doesn't contain an obstacle
random_position = np.random.randint(0, grid.shape[0], 2)
while grid[random_position[0], random_position[1]] == 1:
    print("next random")
    random_position = np.random.randint(0, grid.shape[0], 2)

grid_goal = (random_position[0], random_position[1])
# TODO: adapt to set goal as latitude / longitude position and convert

# Run A* to find a path from start to goal
# TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
# or move to a different search space such as a graph (not done here)
print('Local Start and Goal: ', grid_start, grid_goal)

path = a_star(grid, grid_start, grid_goal)
pruned_path = prune(path)
print(len(path))
print(len(pruned_path))
draw_grid(grid, [], grid_start, grid_goal, path, pruned_path)


# import cProfile, pstats, io
# from pstats import SortKey

# with cProfile.Profile() as pr:
#     path = a_star(grid, grid_start, grid_goal)
#     s = io.StringIO()
#     sortby = SortKey.CUMULATIVE
#     ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
#     ps.print_stats()
#     print(s.getvalue())

