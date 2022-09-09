import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits import mplot3d
# from numba import jit
import time
import sys

# Define direction codes
UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3
# For testing
# Obstacles are stored as tuples of (x, y, orientation)
#   - x, y are the cell in which the object is located. (0, 0) is at lower left corner
#   - orient is the direction
obstacles = eval(sys.argv[1])
print(obstacles)
# obstacles = [(4, 10, 3), (12, 3, 3), (3, 10, 0), (5, 3, 3), (3, 17, 3)] # For testing

cost_grid = np.full((20, 20, 4), 10000000000, dtype=int)
prev_grid = np.zeros((20, 20, 4, 3), dtype=int)
coll_grid = np.zeros((20, 20), dtype=int)



def get_disc_collision_mask():
    for obstacle in obstacles:
        coll_grid[int(obstacle[0])][int(obstacle[1])] = 1
    return coll_grid

# Creates adgacency matrix of traversable points within the 20x20 grid with 4 possible orientations.
# @jit(nopython=True)
def create_graph():
    edges = np.zeros((20, 20, 4, 20, 20, 4), dtype=int)
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
def get_stop_pos(obst):
    if obst[2] == UP:
        return (obst[0], obst[1] + 3, DOWN)
    elif obst[2] == DOWN:
        return (obst[0], obst[1] - 3, UP)
    elif obst[2] == LEFT:
        return (obst[0] - 3, obst[1], RIGHT)
    elif obst[2] == RIGHT:
        return (obst[0] + 3, obst[1], LEFT)
coll_grid = get_disc_collision_mask()
edges = create_graph()

# @jit(nopython=True)
def discrete_pathfind(start_pos, end_pos):
    p_queue = [(0, start_pos, (-1, -1, -1))]
    heapq.heapify(p_queue)
    while(len(p_queue) > 0):
        cur = heapq.heappop(p_queue)
        cost = cur[0]
        cur_pos = cur[1]
        prev = cur[2]

        # print(cur[1])
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
                            heapq.heappush(p_queue, ((cost + edge), (i,j,theta), cur_pos))
path_x = []
path_y = []
instr = []
start = time.time()
for i in range(len(obstacles) - 1):
    # for j in range(len(obstacles)):
    cost_grid = np.full((20, 20, 4), 10000000000, dtype=int)
    prev_grid = np.zeros((20, 20, 4, 3), dtype=int)
    start_pos = get_stop_pos(obstacles[i])
    end_pos = get_stop_pos(obstacles[i+1])
    cost, prev = discrete_pathfind(end_pos, start_pos)

    cur = prev
    prev = start_pos
    path_x.append(prev[0] + 0.5)
    path_y.append(prev[1] + 0.5)
    while cur != (-1, -1, -1):
        if prev[2] == UP:
            if cur[2] == UP:
                if cur[1] - prev[1] > 0:
                    instr.append("F" + str((cur[1] - prev[1])*10))
                else:
                    instr.append("B" + str((prev[1] - cur[1])*10))
            elif cur[2] == LEFT:
                if cur[1] > prev[1]: 
                    instr.append("L" + str((cur[1] - prev[1])*10))
                else:
                    instr.append("R" + str((cur[1] - prev[1])*10))
            elif cur[2] == RIGHT:
                if cur[1] > prev[1]:
                    instr.append("R" + str((cur[1] - prev[1])*10))
                else:
                    instr.append("L" + str((cur[1] - prev[1])*10))
        elif prev[2] == DOWN:
            if cur[2] == DOWN:
                if cur[1] - prev[1] > 0:
                    instr.append("B" + str((cur[1] - prev[1])*10))
                else:
                    instr.append("F" + str((prev[1] - cur[1])*10))
            elif cur[2] == LEFT:
                if prev[1] > cur[1]:
                    instr.append("R" + str((prev[1] - cur[1])*10))
                else:
                    instr.append("L" + str((prev[1] - cur[1])*10))
            elif cur[2] == RIGHT:
                if prev[1] > cur[1]:
                    instr.append("L" + str((prev[1] - cur[1])*10))
                else:
                    instr.append("R" + str((prev[1] - cur[1])*10))
        elif prev[2] == LEFT:
            if cur[2] == LEFT:
                if cur[0] - prev[0] > 0:
                    instr.append("B" + str((cur[0] - prev[0])*10))
                else:
                    instr.append("F" + str((prev[0] - cur[0])*10))
            elif cur[2] == UP:
                if prev[0] > cur[0]:
                    instr.append("R" + str((prev[0] - cur[0])*10))
                else:
                    instr.append("L" + str((prev[0] - cur[0])*10))
            elif cur[2] == DOWN:
                if prev[0] > cur[0]:
                    instr.append("L" + str((prev[0] - cur[0])*10))
                else:
                    instr.append("R" + str((prev[0] - cur[0])*10))
        elif prev[2] == RIGHT:
            if cur[2] == RIGHT:
                if cur[0] - prev[0] > 0:
                    instr.append("F" + str((cur[0] - prev[0])*10))
                else:
                    instr.append("B" + str((prev[0] - cur[0])*10))
            elif cur[2] == UP:
                if cur[0] > prev[0]:
                    instr.append("L" + str((cur[0] - prev[0])*10))
                else:
                    instr.append("R" + str((cur[0] - prev[0])*10))
            elif cur[2] == DOWN:
                if cur[0] > prev[0]:
                    instr.append("R" + str((cur[0] - prev[0])*10))
                else:
                    instr.append("L" + str((cur[0] - prev[0])*10))
        path_x.append(cur[0] + 0.5)
        path_y.append(cur[1] + 0.5)
        # print(cur)
        prev = cur
        cur = (prev_grid[cur[0]][cur[1]][cur[2]][0], prev_grid[cur[0]][cur[1]][cur[2]][1], prev_grid[cur[0]][cur[1]][cur[2]][2])
    instr.append("S")
    # ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    # ax.set_xlim([0, 20])
    # ax.set_ylim([0, 20])

    # plt.plot(path_x, path_y)

    # for obstacle in obstacles:
    #     ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
    # for point in prev_path:
    #     ax.annotate("", xy=(point[0] + math.sin(math.radians(point[2]))*2, point[1] + math.cos(math.radians(point[2]))*2), xytext=(point[0], point[1]),
    #     arrowprops=dict(arrowstyle="->"))

    plt.show()
print(instr)
print("Time taken: " + str(time.time() - start) + " sec")
# import socket

HOST = '192.168.28.28' # Enter IP or Hostname of your server
PORT = 25000 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

#Lets loop awaiting for your input
command = (instr[0]).encode('utf-8')
s.send(command)
s.close()


print(reply)
