import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits import mplot3d
import numba
from numba import jit
import time
import sys
import itertools

####### TO-DO #######
# - Implement function for finding optimal obstacle order to go to
# - Optimize instruction code (group same consecutive operations into a single one e.g. FW10 FW10 FW10 to FW30)
# - Rewrite code to be compatible with numba
# - Get rid of unnecessary global variables


# Define direction codes
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

# Returns a 2D grid with 1s and 0s indicating where obstacles occupy
def get_disc_collision_mask():
    coll_grid = np.zeros((20, 20), dtype=int)
    for obstacle in obstacles:
        coll_grid[int(obstacle[0])][int(obstacle[1])] = 1
    return coll_grid

# Creates adgacency matrix of traversable points within the 20x20 grid with 4 possible orientations.
@jit(nopython=True)       # Need to fix func later to make it compatible with numba
def create_graph(coll_grid):
    edges = np.zeros((20, 20, 4, 20, 20, 4))
    for idx_x in range(0, 20):
        for idx_y in range(0, 20):
            for idx_theta in range(0, 4):
                # Check for collisions
                # Robot is facing up
                if idx_theta == UP and idx_x >= 1 and idx_x < 19:
                    # Forward
                    if idx_y < 17:
                        if not (coll_grid[idx_x-1][idx_y+3] or coll_grid[idx_x][idx_y+3] or coll_grid[idx_x+1][idx_y+3]):
                            edges[idx_x][idx_y][UP][idx_x][idx_y+1][UP] = 1

                            # Forward turn right
                            if idx_x < 16:
                                if not (coll_grid[idx_x+2][idx_y+3] or coll_grid[idx_x+3][idx_y+3] or coll_grid[idx_x+4][idx_y+3] or \
                                    coll_grid[idx_x+2][idx_y+2] or coll_grid[idx_x+3][idx_y+2] or coll_grid[idx_x+4][idx_y+2] or \
                                    coll_grid[idx_x+2][idx_y+1] or coll_grid[idx_x+3][idx_y+1] or coll_grid[idx_x+4][idx_y+1]):
                                        edges[idx_x][idx_y][idx_theta][idx_x+2][idx_y+2][RIGHT] = 5

                            # Forward turn left
                            if idx_x >= 4:
                                 if not (coll_grid[idx_x-2][idx_y+3] or coll_grid[idx_x-3][idx_y+3] or coll_grid[idx_x-4][idx_y+3] or \
                                    coll_grid[idx_x-2][idx_y+2] or coll_grid[idx_x-3][idx_y+2] or coll_grid[idx_x-4][idx_y+2] or \
                                    coll_grid[idx_x-2][idx_y+1] or coll_grid[idx_x-3][idx_y+1] or coll_grid[idx_x-4][idx_y+1]):
                                        edges[idx_x][idx_y][idx_theta][idx_x-2][idx_y+2][LEFT] = 5
                    # Backward
                    if idx_y >= 1:
                        if not (coll_grid[idx_x-1][idx_y-1] or coll_grid[idx_x][idx_y-1] or coll_grid[idx_x+1][idx_y-1]):
                            edges[idx_x][idx_y][UP][idx_x][idx_y-1][UP] = 1

                    if idx_y >= 3:
                        if not (coll_grid[idx_x-1][idx_y-1] or coll_grid[idx_x][idx_y-1] or coll_grid[idx_x+1][idx_y-1] or \
                            coll_grid[idx_x-1][idx_y-2] or coll_grid[idx_x][idx_y-2] or coll_grid[idx_x+1][idx_y-2] or \
                            coll_grid[idx_x-1][idx_y-3] or coll_grid[idx_x][idx_y-3] or coll_grid[idx_x+1][idx_y-3]):
                            # Backward turn right
                            if idx_x < 18:
                                if not (coll_grid[idx_x+2][idx_y-1] or coll_grid[idx_x+2][idx_y-2] or coll_grid[idx_x+2][idx_y-3]):
                                    edges[idx_x][idx_y][UP][idx_x+2][idx_y-2][LEFT] = 5

                            # Backward turn left
                            if idx_x >= 2:
                                if not (coll_grid[idx_x-2][idx_y-1] or coll_grid[idx_x-2][idx_y-2] or coll_grid[idx_x-2][idx_y-3]):
                                    edges[idx_x][idx_y][UP][idx_x-2][idx_y-2][RIGHT] = 5

                # Robot is facing down
                if idx_theta == DOWN and idx_x >= 1 and idx_x < 19:
                    # Forward
                    if idx_y >= 3:
                        if not (coll_grid[idx_x-1][idx_y-3] or coll_grid[idx_x][idx_y-3] or coll_grid[idx_x+1][idx_y-3]):
                            edges[idx_x][idx_y][DOWN][idx_x][idx_y-1][DOWN] = 1

                            # Forward turn right
                            if idx_x >= 4:
                                 if not (coll_grid[idx_x-2][idx_y-3] or coll_grid[idx_x-3][idx_y-3] or coll_grid[idx_x-4][idx_y-3] or \
                                    coll_grid[idx_x-2][idx_y-2] or coll_grid[idx_x-3][idx_y-2] or coll_grid[idx_x-4][idx_y-2] or \
                                    coll_grid[idx_x-2][idx_y-1] or coll_grid[idx_x-3][idx_y-1] or coll_grid[idx_x-4][idx_y-1]):
                                        edges[idx_x][idx_y][idx_theta][idx_x-2][idx_y-2][LEFT] = 5

                            # Forward turn left
                            if idx_x < 16:
                                if not (coll_grid[idx_x+2][idx_y-3] or coll_grid[idx_x+3][idx_y-3] or coll_grid[idx_x+4][idx_y-3] or \
                                    coll_grid[idx_x+2][idx_y-2] or coll_grid[idx_x+3][idx_y-2] or coll_grid[idx_x+4][idx_y-2] or \
                                    coll_grid[idx_x+2][idx_y-1] or coll_grid[idx_x+3][idx_y-1] or coll_grid[idx_x+4][idx_y-1]):
                                        edges[idx_x][idx_y][idx_theta][idx_x+2][idx_y-2][RIGHT] = 5
                            
                    # Backward
                    if idx_y < 19:
                        if not (coll_grid[idx_x-1][idx_y+1] or coll_grid[idx_x][idx_y+1] or coll_grid[idx_x+1][idx_y+1]):
                            edges[idx_x][idx_y][DOWN][idx_x][idx_y+1][DOWN] = 1

                    if idx_y < 17:
                        if not (coll_grid[idx_x-1][idx_y+1] or coll_grid[idx_x][idx_y+1] or coll_grid[idx_x+1][idx_y+1] or \
                            coll_grid[idx_x-1][idx_y+2] or coll_grid[idx_x][idx_y+2] or coll_grid[idx_x+1][idx_y+2] or \
                            coll_grid[idx_x-1][idx_y+3] or coll_grid[idx_x][idx_y+3] or coll_grid[idx_x+1][idx_y+3]):
                            # Backward turn right
                            if idx_x >= 2:
                                if not (coll_grid[idx_x-2][idx_y+1] or coll_grid[idx_x-2][idx_y+2] or coll_grid[idx_x-2][idx_y+3]):
                                    edges[idx_x][idx_y][DOWN][idx_x-2][idx_y+2][RIGHT] = 5

                            # Backward turn left
                            if idx_x < 18:
                                if not (coll_grid[idx_x+2][idx_y+1] or coll_grid[idx_x+2][idx_y+2] or coll_grid[idx_x+2][idx_y+3]):
                                    edges[idx_x][idx_y][DOWN][idx_x+2][idx_y+2][LEFT] = 5

                # Robot is facing left
                if idx_theta == LEFT and idx_y >= 1 and idx_y < 19:
                    # Forward
                    if idx_x >= 3:
                        if not (coll_grid[idx_x-3][idx_y-1] or coll_grid[idx_x-3][idx_y] or coll_grid[idx_x-3][idx_y+1]):
                            edges[idx_x][idx_y][LEFT][idx_x-1][idx_y][LEFT] = 1

                            # Forward turn right
                            if idx_y < 16:
                                if not (coll_grid[idx_x-3][idx_y+2] or coll_grid[idx_x-3][idx_y+3] or coll_grid[idx_x-3][idx_y+4] or \
                                    coll_grid[idx_x-2][idx_y+2] or coll_grid[idx_x-2][idx_y+3] or coll_grid[idx_x-2][idx_y+4] or \
                                    coll_grid[idx_x-1][idx_y+2] or coll_grid[idx_x-1][idx_y+3] or coll_grid[idx_x-1][idx_y+4]):
                                        edges[idx_x][idx_y][idx_theta][idx_x-2][idx_y+2][UP] = 5

                            # Forward turn left
                            if idx_y >= 4:
                                 if not (coll_grid[idx_x-3][idx_y-2] or coll_grid[idx_x-3][idx_y-3] or coll_grid[idx_x-3][idx_y-4] or \
                                    coll_grid[idx_x-2][idx_y-2] or coll_grid[idx_x-2][idx_y-3] or coll_grid[idx_x-2][idx_y-4] or \
                                    coll_grid[idx_x-1][idx_y-2] or coll_grid[idx_x-1][idx_y-3] or coll_grid[idx_x-1][idx_y-4]):
                                        edges[idx_x][idx_y][idx_theta][idx_x-2][idx_y-2][DOWN] = 5
                            
                    # Backward
                    if idx_x < 19:
                        if not (coll_grid[idx_x+1][idx_y-1] or coll_grid[idx_x+1][idx_y] or coll_grid[idx_x+1][idx_y+1]):
                            edges[idx_x][idx_y][LEFT][idx_x+1][idx_y][LEFT] = 1

                    if idx_x < 17:
                        if not (coll_grid[idx_x+1][idx_y-1] or coll_grid[idx_x+1][idx_y] or coll_grid[idx_x+1][idx_y+1] or \
                            coll_grid[idx_x+2][idx_y-1] or coll_grid[idx_x+2][idx_y] or coll_grid[idx_x+2][idx_y+1] or \
                            coll_grid[idx_x+3][idx_y-1] or coll_grid[idx_x+3][idx_y] or coll_grid[idx_x+3][idx_y+1]):
                            # Backward turn right
                            if idx_y < 18:
                                if not (coll_grid[idx_x+1][idx_y+2] or coll_grid[idx_x+2][idx_y+2] or coll_grid[idx_x+3][idx_y+2]):
                                    edges[idx_x][idx_y][LEFT][idx_x+2][idx_y+2][DOWN] = 5

                            # Backward turn left
                            if idx_y >= 2:
                                if not (coll_grid[idx_x+1][idx_y-2] or coll_grid[idx_x+2][idx_y-2] or coll_grid[idx_x+3][idx_y-2]):
                                    edges[idx_x][idx_y][LEFT][idx_x+2][idx_y-2][UP] = 5

                # Robot is facing right
                if idx_theta == RIGHT and idx_y >= 1 and idx_y < 19:
                    # Forward
                    if idx_x < 17:
                        if not (coll_grid[idx_x+3][idx_y-1] or coll_grid[idx_x+3][idx_y] or coll_grid[idx_x+3][idx_y+1]):
                            edges[idx_x][idx_y][RIGHT][idx_x+1][idx_y][RIGHT] = 1

                            # Forward turn right
                            if idx_y >= 4:
                                 if not (coll_grid[idx_x+3][idx_y-2] or coll_grid[idx_x+3][idx_y-3] or coll_grid[idx_x+3][idx_y-4] or \
                                    coll_grid[idx_x+2][idx_y-2] or coll_grid[idx_x+2][idx_y-3] or coll_grid[idx_x+2][idx_y-4] or \
                                    coll_grid[idx_x+1][idx_y-2] or coll_grid[idx_x+1][idx_y-3] or coll_grid[idx_x+1][idx_y-4]):
                                        edges[idx_x][idx_y][idx_theta][idx_x+2][idx_y-2][DOWN] = 5

                            # Forward turn left
                            if idx_y < 16:
                                if not (coll_grid[idx_x+3][idx_y+2] or coll_grid[idx_x+3][idx_y+3] or coll_grid[idx_x+3][idx_y+4] or \
                                    coll_grid[idx_x+2][idx_y+2] or coll_grid[idx_x+2][idx_y+3] or coll_grid[idx_x+2][idx_y+4] or \
                                    coll_grid[idx_x+1][idx_y+2] or coll_grid[idx_x+1][idx_y+3] or coll_grid[idx_x+1][idx_y+4]):
                                        edges[idx_x][idx_y][idx_theta][idx_x+2][idx_y+2][UP] = 5

                    # Backward
                    if idx_x >= 1:
                        if not (coll_grid[idx_x-1][idx_y-1] or coll_grid[idx_x-1][idx_y] or coll_grid[idx_x-1][idx_y+1]):
                            edges[idx_x][idx_y][RIGHT][idx_x-1][idx_y][RIGHT] = 1

                    if idx_x >= 3:
                        if not (coll_grid[idx_x-1][idx_y-1] or coll_grid[idx_x-1][idx_y] or coll_grid[idx_x-1][idx_y+1] or \
                            coll_grid[idx_x-2][idx_y-1] or coll_grid[idx_x-2][idx_y] or coll_grid[idx_x-2][idx_y+1] or \
                            coll_grid[idx_x-3][idx_y-1] or coll_grid[idx_x-3][idx_y] or coll_grid[idx_x-3][idx_y+1]):

                            # Backward turn right
                            if idx_y >= 2:
                                if not (coll_grid[idx_x-1][idx_y-2] or coll_grid[idx_x-2][idx_y-2] or coll_grid[idx_x-3][idx_y-2]):
                                    edges[idx_x][idx_y][RIGHT][idx_x-2][idx_y-2][UP] = 5
                            # Backward turn left

                            if idx_y < 18:
                                if not (coll_grid[idx_x-1][idx_y+2] or coll_grid[idx_x-2][idx_y+2] or coll_grid[idx_x-3][idx_y+2]):
                                    edges[idx_x][idx_y][RIGHT][idx_x-2][idx_y+2][DOWN] = 5
    return edges;

# Returns the stopping position of the robot for the obstacle
@jit(nopython=True)
def get_stop_pos(obst):
    if obst[2] == UP:
        return (obst[0], obst[1] + 3, DOWN)
    elif obst[2] == DOWN:
        return (obst[0], obst[1] - 3, UP)
    elif obst[2] == LEFT:
        return (obst[0] - 3, obst[1], RIGHT)
    elif obst[2] == RIGHT:
        return (obst[0] + 3, obst[1], LEFT)

@jit(nopython=True)           # Need to fix func later to make it compatible with numba
def discrete_pathfind(start_pos, end_pos, cost_grid, prev_grid):
    p_queue = [(0, start_pos, (-1, -1, -1))]
    heapq.heapify(p_queue)
    while(len(p_queue) > 0):
        cur = heapq.heappop(p_queue)
        cost = cur[0]
        cur_pos = cur[1]
        prev = cur[2]

        if(cur_pos == end_pos):
            return cost, prev

        if(cost_grid[cur_pos[0]][cur_pos[1]][cur_pos[2]] > cost):
            cost_grid[cur_pos[0]][cur_pos[1]][cur_pos[2]] = cost
            for i in range(3):
                prev_grid[cur_pos[0]][cur_pos[1]][cur_pos[2]][i] = prev[i]

            for i in range(0, 20):
                for j in range(0, 20):
                    for theta in range(0, 4):
                        edge = edges[cur_pos[0]][cur_pos[1]][cur_pos[2]][i][j][theta]
                        if edge > 0:
                            heapq.heappush(p_queue, (int(cost + edge), (i,j,theta), cur_pos))
# Returns a list of all the possible orders for numbers from 1 to num in a list
def generate_permutations(num):
    return list(itertools.permutations([x for x in range(1, num + 1)]))


def get_distance_graph(obstacles, coll_grid, init_pos):
    # permu_list = [[0] + list(x) for x in generate_permutations(len(obstacles))]

    # print(permu_list)
    cost_matrix = np.zeros((len(obstacles) + 1, len(obstacles)))
    path_matrix = []

    path_x = []
    path_y = []
    instr = []
    print(obstacles)
    for i in range(len(obstacles) + 1):
        path_list = []
        for j in range(0, len(obstacles)):
            instr = []
            path_x = []
            path_y = []
            cost_grid = np.full((20, 20, 4), 2147483647)
            prev_grid = np.zeros((20, 20, 4, 3))
            # for j in range(len(obstacles)):
            if i == 0:
                start_pos = init_pos
            else:
                start_pos = get_stop_pos(obstacles[i - 1])

            end_pos = get_stop_pos(obstacles[j])
            dest_str = str(obstacles[j ][0]).zfill(2) + str(obstacles[j][1]).zfill(2)

            instr.append('PR|O' + dest_str)


            cost, prev = discrete_pathfind(end_pos, start_pos, cost_grid, prev_grid)

            cur = prev
            prev = start_pos
            path_x.append(prev[0] + 0.5)
            path_y.append(prev[1] + 0.5)
            while cur != (-1, -1, -1):
                if prev[2] == UP:
                    if cur[2] == UP:
                        if cur[1] - prev[1] > 0:
                            heading = "FW"
                            dist = int((cur[1] - prev[1])*10)
                        else:
                            heading = "BW"
                            dist = int((prev[1] - cur[1])*10)
                    elif cur[2] == LEFT:
                        if cur[1] > prev[1]:
                            heading = "FL"
                            dist = int((cur[1] - prev[1])*10)
                        else:
                            heading = "BR"
                            dist = int((prev[1] - cur[1])*10)
                    elif cur[2] == RIGHT:
                        if cur[1] > prev[1]:
                            heading = "FR"
                            dist = int((cur[1] - prev[1])*10)
                        else:
                            heading = "BL"
                            dist = int((prev[1] - cur[1])*10)
                elif prev[2] == DOWN:
                    if cur[2] == DOWN:
                        if cur[1] - prev[1] > 0:
                            heading = "BW"
                            dist = int((cur[1] - prev[1])*10)
                        else:
                            heading = "FW"
                            dist = int((prev[1] - cur[1])*10)
                    elif cur[2] == LEFT:
                        if prev[1] > cur[1]:
                            heading = "FR"
                            dist = int((prev[1] - cur[1])*10)
                        else:
                            heading = "BL"
                            dist = int((cur[1] - prev[1])*10)
                    elif cur[2] == RIGHT:
                        if prev[1] > cur[1]:
                            heading = "FL"
                            dist = int((prev[1] - cur[1])*10)
                        else:
                            heading = "BR"
                            dist = int((cur[1] - prev[1])*10)
                elif prev[2] == LEFT:
                    if cur[2] == LEFT:
                        if cur[0] - prev[0] > 0:
                            heading = "BW"
                            dist = int((cur[0] - prev[0])*10)
                        else:
                            heading = "FW"
                            dist = int((prev[0] - cur[0])*10)
                    elif cur[2] == UP:
                        if prev[0] > cur[0]:
                            heading = "FR"
                            dist = int((prev[0] - cur[0])*10)
                        else:
                            heading = "BL"
                            dist = int((cur[0] - prev[0])*10)
                    elif cur[2] == DOWN:
                        if prev[0] > cur[0]:
                            heading = "FL"
                            dist = int((prev[0] - cur[0])*10)
                        else:
                            heading = "BR"
                            dist = int((cur[0] - prev[0])*10)
                elif prev[2] == RIGHT:
                    if cur[2] == RIGHT:
                        if cur[0] - prev[0] > 0:
                            heading = "FW"
                            dist = int((cur[0] - prev[0])*10)
                        else:
                            heading = "BW"
                            dist = int((prev[0] - cur[0])*10)
                    elif cur[2] == UP:
                        if cur[0] > prev[0]:
                            heading = "FL"
                            dist = int((cur[0] - prev[0])*10)
                        else:
                            heading = "BR"
                            dist = int((prev[0] - cur[0])*10)
                    elif cur[2] == DOWN:
                        if cur[0] > prev[0]:
                            heading = "FR"
                            dist = int((cur[0] - prev[0])*10)
                        else:
                            heading = "BL"
                            dist = int((prev[0] - cur[0])*10)

                instr.append("PS|" + heading + str(dist).zfill(3))

                path_x.append(cur[0] + 0.5)
                path_y.append(cur[1] + 0.5)
                # print(cur)
                prev = cur
                cur = (prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][0], prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][1], prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][2])
            
            instr.append("PR|S" + dest_str)

            # print(instr)

            # ax = plt.gca()
            # ax.set_aspect('equal', adjustable='box')
            # ax.set_xlim([0, 20])
            # ax.set_ylim([0, 20])

            # plt.plot(path_x, path_y)

            # for obstacle in obstacles:
            #     ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
            # plt.show()

            path_list.append(instr)
            cost_matrix[i][j] = cost
        path_matrix.append(path_list)
    # print(cost_matrix)
            

    ###### Uncomment below to plot path in matplotlib ######

    
    return cost_matrix, path_matrix


def plot_instr(ptr, instr, start_pos):
    pass

# Obstacles are stored as tuples of (x, y, orientation)
#   - x, y are the cell in which the object is located. (0, 0) is at lower left corner
#   - orient is the direction
obstacles = eval(sys.argv[1])   # Read obstacle string

init_pos = eval(sys.argv[2])
# print(obstacles)

start = time.time()
# obstacles = [(4, 10, 3), (12, 3, 3), (3, 10, 0), (5, 3, 3), (3, 17, 3)] # For testing
print("Obstacles loaded - %.3fsec" % (time.time() - start))

start = time.time()
coll_grid = get_disc_collision_mask()
edges = create_graph(coll_grid)
print("Graph created - %.3fsec" % (time.time() - start))


start = time.time()
cost_matrix, path_matrix = get_distance_graph(obstacles, coll_grid, init_pos)
instr_list = []
for i in range(len(obstacles)):
    instr_list += path_matrix[i][i]
print(instr_list)

## Simplify instructions 



print("Path generated - %.3fsec" % (time.time() - start))


# Still sorta broken
# import socket

# HOST = '192.168.28.28' # Enter IP or Hostname of your server
# PORT = 25000 # Pick an open Port (1000+ recommended), must match the server port
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST,PORT))

# #Lets loop awaiting for your input
# command = (instr[0]).encode('utf-8')
# s.send(command)
# s.close()


# print(reply)
