import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc
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

TURNING_RAD = 1.5

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
@jit(nopython=True)
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

@jit(nopython=True)
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


# @jit(nopython=True)
# def using_numba(pool, r):
#     n = len(pool)
#     indices = list(range(r))
#     empty = not(n and (0 < r <= n))

#     if not empty:
#         result = [pool[i] for i in indices]
#         yield result

#     while not empty:
#         i = r - 1
#         while i >= 0 and indices[i] == i + n - r:
#             i -= 1
#         if i < 0:
#             empty = True
#         else:
#             indices[i] += 1
#             for j in range(i+1, r):
#                 indices[j] = indices[j-1] + 1

#             result = [pool[i] for i in indices]
#             yield result

# Returns a list of all the possible orders for numbers from 1 to num in a list
@jit(nopython=True)
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
                            dist = int((cur[1] - prev[1])*45)
                        else:
                            heading = "BR"
                            dist = int((prev[1] - cur[1])*45)
                    elif cur[2] == RIGHT:
                        if cur[1] > prev[1]:
                            heading = "FR"
                            dist = int((cur[1] - prev[1])*45)
                        else:
                            heading = "BL"
                            dist = int((prev[1] - cur[1])*45)
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
                            dist = int((prev[1] - cur[1])*45)
                        else:
                            heading = "BL"
                            dist = int((cur[1] - prev[1])*45)
                    elif cur[2] == RIGHT:
                        if prev[1] > cur[1]:
                            heading = "FL"
                            dist = int((prev[1] - cur[1])*45)
                        else:
                            heading = "BR"
                            dist = int((cur[1] - prev[1])*45)
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
                            dist = int((prev[0] - cur[0])*45)
                        else:
                            heading = "BL"
                            dist = int((cur[0] - prev[0])*45)
                    elif cur[2] == DOWN:
                        if prev[0] > cur[0]:
                            heading = "FL"
                            dist = int((prev[0] - cur[0])*45)
                        else:
                            heading = "BR"
                            dist = int((cur[0] - prev[0])*45)
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
                            dist = int((cur[0] - prev[0])*45)
                        else:
                            heading = "BR"
                            dist = int((prev[0] - cur[0])*45)
                    elif cur[2] == DOWN:
                        if cur[0] > prev[0]:
                            heading = "FR"
                            dist = int((cur[0] - prev[0])*45)
                        else:
                            heading = "BL"
                            dist = int((prev[0] - cur[0])*45)
                if heading == 'FR':
                    instr.append("PS|BW003")
                elif heading == 'BR':
                    instr.append("PS|FW003")
                instr.append("PS|" + heading + str(dist).zfill(3))
                if heading == 'FR':
                    instr.append("PS|BW004")
                elif heading == 'BR':
                    instr.append("PS|FW004")

                path_x.append(cur[0] + 0.5)
                path_y.append(cur[1] + 0.5)
                # print(cur)
                prev = cur
                cur = (prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][0], prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][1], prev_grid[int(cur[0])][int(cur[1])][int(cur[2])][2])
            
            instr.append("PR|S" + dest_str)

            # print(instr)

            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlim([0, 20])
            ax.set_ylim([0, 20])

            # plt.plot(path_x, path_y)

            for obstacle in obstacles:
                ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
            # plt.show()

            path_list.append(instr)
            cost_matrix[i][j] = cost
        path_matrix.append(path_list)
    # print(cost_matrix)
    
    return cost_matrix, path_matrix
@jit(nopython=True)
def get_path_instructions(cost_matrix, path_matrix, num_obst):
    generate_permutations(num_obst)

# For plotting paths (with actual arcs), still WIP
def plot_instr(plt, instr_list, start_pos, obstacles):
    x = []
    y = []
    x.append(start_pos[0])
    y.append(start_pos[1])
    cur_pos = start_pos
    for instr in instr_list:
        instr_heading = instr[3:5]
        instr_dist = int(instr[5:8])
        if instr_heading == 'FW':
            if cur_pos[2] == UP:
                cur_pos = (cur_pos[0], cur_pos[1] + instr_dist, cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == DOWN:
                cur_pos = (cur_pos[0], cur_pos[1] - instr_dist, cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == LEFT:
                cur_pos = (cur_pos[0] - instr_dist, cur_pos[1], cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == RIGHT:
                cur_pos = (cur_pos[0] + instr_dist, cur_pos[1], cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])

        elif instr_heading == 'BW':
            if cur_pos[2] == UP:
                cur_pos = (cur_pos[0], cur_pos[1] - instr_dist, cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == DOWN:
                cur_pos = (cur_pos[0], cur_pos[1] + instr_dist, cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == LEFT:
                cur_pos = (cur_pos[0] + instr_dist, cur_pos[1], cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])
            elif cur_pos[2] == RIGHT:
                cur_pos = (cur_pos[0] - instr_dist, cur_pos[1], cur_pos[2])
                x.append(cur_pos[0])
                y.append(cur_pos[1])

        elif instr_heading == 'FL':
            if cur_pos[2] == UP:
                turning_center = cur_pos[0] - TURNING_RAD, cur_pos[1]
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim([0, 20])
    ax.set_ylim([0, 20])

    # plt.plot(x, y)

    for obstacle in obstacles:
        ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
    # plt.show()

# Simplify instructions list
def simplify_instr(instr_list):
    temp_instr_list = [instr_list[0]]
    index = 1
    instr_len = len(instr_list)
    while index < instr_len:
        # while the next instr is of the same type, make sure its not a Stop or Obstacle instr
        if instr_list[index][3] != "S" and instr_list[index][3] != "O" and index < instr_len and instr_list[index][:5] == temp_instr_list[-1][:5]:
            prev_instr = temp_instr_list.pop()
            # add prev and current dist
            new_dist = int(instr_list[index][5:]) + int(prev_instr[5:])
            if new_dist < 100:
                new_dist = "0" + str(new_dist)
            else:
                new_dist = str(new_dist)
            # add new instr string
            temp_instr_list.append(prev_instr[:5] + new_dist)
        else:
            temp_instr_list.append(instr_list[index])
        index += 1
    return temp_instr_list

def optimal_path(obstacles, cost_matrix, path_matrix):
    visited = set()
    target = len(obstacles) + 1 # visit all obstacles, and starting point
    min_cost = float("inf")
    optimal_path = []
    instr_list = []

    def path_cost(obstacle, cur_cost, cur_path):
        nonlocal min_cost
        nonlocal optimal_path
        visited.add(obstacle)

        if len(visited) == target:
            if cur_cost < min_cost:
                min_cost = cur_cost
                optimal_path = cur_path[:]        
        else:
            for next_obstacle in range(1, len(cost_matrix)):     # len(cost_matrix) -> n + 1
                if obstacle != next_obstacle and next_obstacle not in visited:    # inner array: costs from current to obstacle (0-indexed), outer array: obstacle (1-indexed)
                    cur_path.append((obstacle, next_obstacle))
                    path_cost(next_obstacle, cur_cost + cost_matrix[obstacle][next_obstacle-1], cur_path)
                    cur_path.pop()
        visited.remove(obstacle)

    path_cost(0, 0, [])
    print("Optimal path: ", optimal_path, "with cost:", min_cost)

    for from_obstacle, to_obstacle in optimal_path:
        for instr in path_matrix[from_obstacle][to_obstacle-1]:
            instr_list.append(instr)
    
    return instr_list


start = time.time()
# Obstacles are stored as tuples of (x, y, orientation)
#   - x, y are the cell in which the object is located. (0, 0) is at lower left corner
#   - orient is the direction

obstacles = []
# obstacles = eval(sys.argv[1])   # Read obstacle string
# # obstacles = [(4, 10, 3), (12, 3, 3), (3, 10, 0), (5, 3, 3), (3, 17, 3)] # For testing
# init_pos = eval(sys.argv[2])    # Read starting position string
# # print(obstacles)

# print("Obstacles loaded - %.3fsec" % (time.time() - start))

# start = time.time()
# coll_grid = get_disc_collision_mask()
# edges = create_graph(coll_grid)
# print("Graph created - %.3fsec" % (time.time() - start))

# start = time.time()
# cost_matrix, path_matrix = get_distance_graph(obstacles, coll_grid, init_pos)
# print("Obstacle visit paths generated - %.3fsec" % (time.time() - start))

# start = time.time()

# instr_list = optimal_path(obstacles, cost_matrix, path_matrix)

# # for i in range(len(obstacles)):
# #     instr_list += path_matrix[i][i]
# # instr_list = get_path_instructions(cost_matrix, path_matrix, len(obstacles))

# instr_list = simplify_instr(instr_list)
# print(instr_list)
# instr_list = ['PS|FW050','PS|BR100','PS|FW020','PS|FR100','PS|FR100','PS|FW020','PS|FR100','PS|FR100','PS|BW040','PS|FL038','PS|FW020']
instr_list = ['PR|O1918', 'PS|FW160', 'PS|BW003', 'PS|FR090', 'PS|BW004', 'PS|FW120', 'PR|S1918']
# instr_list = ['PS|FR090']
print("Final path instructions generated - %.3fsec" % (time.time() - start))

# For sending data to RPi via TCP socket
import socket

HOST = '192.168.28.28' # RPi IP
PORT = 12345 # Port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

while True:
    reply_raw = s.recv(1024)
    reply = reply_raw.strip().decode('utf-8')
    if reply != '':
        print(reply_raw, reply)
        x_local = int(reply[0:2])
        y_local = int(reply[2:4])
        if reply[4] == '0':
            orient_local = UP
        elif reply[4] == '1':
            orient_local = RIGHT
        elif reply[4] == '2':
            orient_local = DOWN
        elif reply[4] == '3':
            orient_local = LEFT
        id_local = int(reply[5])

        obstacles.append((x_local, y_local, orient_local, id_local))
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim([0, 20])
        ax.set_ylim([0, 20])

        # plt.plot(path_x, path_y)

        for obstacle in obstacles:
            ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
        plt.show()





# Add termination function for RPi to stop recieving
instr_list += ['PR|STOP']
print(instr_list)

# Loop through all the instructions to send to RPi
for i in range(len(instr_list)):
    command = (instr_list[i]+',').encode('utf-8')
    s.send(command)

# while True:



    # reply = s.recv(1024).strip().decode('utf-8')
    # print(reply)
    # if reply == 'OK':
    #     print(i, instr_list[i], 'sent')
    # elif reply == 'Terminate':
    #     print("Sending complete!")
    # else:
    #     print("Reply = " + reply)
    #     print("Unexpected reply encountered, program exiting...")
    #     exit(0)

# FR110
# x = []
# y = []
# cur_pos = init_pos
# for instr in instr_list:
#     x.append(cur_pos[0] + 0.5)
#     y.append(cur_pos[1] + 0.5)
#     if cur_pos[2] == UP:
#         if instr[3:5] == 'FW':
#             cur_pos = (cur_pos[0], cur_pos[1] + int(instr[5:8])/10, cur_pos[2])
#         elif instr[3:5] == 'BW':
#             cur_pos = (cur_pos[0], cur_pos[1] - int(instr[5:8])/10, cur_pos[2])
#         elif instr[3:5] == 'FR':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] + 2, RIGHT)
#         elif instr[3:5] == 'FL':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] + 2, LEFT)
#         elif instr[3:5] == 'BR':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] - 2, LEFT)
#         elif instr[3:5] == 'BL':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] - 2, RIGHT)
#     elif cur_pos[2] == DOWN:
#         if instr[3:5] == 'FW':
#             cur_pos = (cur_pos[0], cur_pos[1] - int(instr[5:8])/10, cur_pos[2])
#         elif instr[3:5] == 'BW':
#             cur_pos = (cur_pos[0], cur_pos[1] + int(instr[5:8])/10, cur_pos[2])
#         elif instr[3:5] == 'FR':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] - 2, LEFT)
#         elif instr[3:5] == 'FL':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] - 2, RIGHT)
#         elif instr[3:5] == 'BR':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] + 2, RIGHT)
#         elif instr[3:5] == 'BL':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] + 2, LEFT)
#     elif cur_pos[2] == LEFT:
#         if instr[3:5] == 'FW':
#             cur_pos = (cur_pos[0] - int(instr[5:8])/10, cur_pos[1], cur_pos[2])
#         elif instr[3:5] == 'BW':
#             cur_pos = (cur_pos[0] + int(instr[5:8])/10, cur_pos[1], cur_pos[2])
#         elif instr[3:5] == 'FR':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] + 2, UP)
#         elif instr[3:5] == 'FL':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] - 2, DOWN)
#         elif instr[3:5] == 'BR':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] + 2, DOWN)
#         elif instr[3:5] == 'BL':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] - 2, UP)
#     elif cur_pos[2] == RIGHT:
#         if instr[3:5] == 'FW':
#             cur_pos = (cur_pos[0] + int(instr[5:8])/10, cur_pos[1], cur_pos[2])
#         elif instr[3:5] == 'BW':
#             cur_pos = (cur_pos[0] - int(instr[5:8])/10, cur_pos[1], cur_pos[2])
#         elif instr[3:5] == 'FR':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] - 2, DOWN)
#         elif instr[3:5] == 'FL':
#             cur_pos = (cur_pos[0] + 2, cur_pos[1] + 2, UP)
#         elif instr[3:5] == 'BR':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] - 2, UP)
#         elif instr[3:5] == 'BL':
#             cur_pos = (cur_pos[0] - 2, cur_pos[1] + 2, DOWN)
#     # print(cur_pos)
# print(x)
# ax = plt.gca()
# ax.set_aspect('equal', adjustable='box')
# ax.set_xlim([0, 20])
# ax.set_ylim([0, 20])

# plt.plot(x, y)

# for obstacle in obstacles:
#     ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))

# plt.show()

# import serial

# ser = serial.Serial()
# ser.baudrate = 115200
# ser.port = 'COM10'
# print(ser.open())
# # if not ser.open():
# #     print("Error opening port!")
# #     exit(0)

# # f = open('instr.txt')
# # instr_list = eval(f.readline())
# # instr_list = ['PS|FL0']
# for instr in instr_list:
#     if instr[0:2] == 'PS':
#         print(instr[3:])
#         ser.write(instr[3:].encode())
#         while True:
#             bytesToRead = ser.inWaiting()
#             raw_dat = ser.read(1)
#             dat = raw_dat.strip().decode()
#             if dat != '':   
#                 print(raw_dat)
#             if dat == 'R':
#                 # exit(0)
#                 break
#         # exit(0)
# print(instr_list)