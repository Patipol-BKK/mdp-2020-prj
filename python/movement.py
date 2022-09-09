import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits import mplot3d
from numba import jit
import time

MATRIX_RES = 10
CELL_SIZE = 200/MATRIX_RES
ANGLE_RES = 16

OBJ_DISTANCE = 30

MIN_COLLISION_DIST = 15

COLLISION_MASK_RES = 2

NUM_OBJ = 5

# @jit(nopython=True)
# def distance(pos_1, pos_2):
#     return math.sqrt((pos_1[0] - pos_2[0])**2 + (pos_1[1] - pos_2[1])**2)

# def get_collision_mask(obstacles):
#     collision_mask = np.zeros((int(200/COLLISION_MASK_RES), int(200/COLLISION_MASK_RES)))
#     for i in range(int(200/COLLISION_MASK_RES)):
#         for j in range(int(200/COLLISION_MASK_RES)):
#             pos_x = i*COLLISION_MASK_RES + COLLISION_MASK_RES/2
#             pos_y = j*COLLISION_MASK_RES + COLLISION_MASK_RES/2
#             for obstacle in obstacles:
#                 corner_topleft = obstacle[0], obstacle[1] + 10
#                 corner_topright = obstacle[0] + 10, obstacle[1] + 10
#                 corner_botleft = obstacle[0], obstacle[1]
#                 corner_botright = obstacle[0] + 10, obstacle[1]

#                 if pos_x >= corner_topleft[0] and pos_x < corner_topright[0]:
#                     if pos_y - corner_topleft[1] <= MIN_COLLISION_DIST and corner_botleft[1] - pos_y <= MIN_COLLISION_DIST:
#                         collision_mask[i][j] = 1
#                         break
#                 elif pos_y >= corner_botleft[1] and pos_y < corner_topleft[1]:
#                     if corner_topleft[0] - pos_x <= MIN_COLLISION_DIST and pos_x - corner_topright[0] <= MIN_COLLISION_DIST:
#                         collision_mask[i][j] = 1
#                         break
#                 elif distance((pos_x, pos_y), corner_topleft) <= MIN_COLLISION_DIST:
#                     collision_mask[i][j] = 1
#                     break
#                 elif distance((pos_x, pos_y), corner_topright) <= MIN_COLLISION_DIST:
#                     collision_mask[i][j] = 1
#                     break
#                 elif distance((pos_x, pos_y), corner_botleft) <= MIN_COLLISION_DIST:
#                     collision_mask[i][j] = 1
#                     break
#                 elif distance((pos_x, pos_y), corner_botright) <= MIN_COLLISION_DIST:
#                     collision_mask[i][j] = 1
#                     break
#     return collision_mask

# @jit(nopython=True)
# def is_valid_pos(pos):
#     if pos[0] <= MIN_COLLISION_DIST or pos[1] <= MIN_COLLISION_DIST or \
#         pos[0] > 200 - MIN_COLLISION_DIST or pos[1] > 200 - MIN_COLLISION_DIST:
#         return False
#     else:
#         if collision_mask[int(math.floor(pos[0]/COLLISION_MASK_RES))][int(math.floor(pos[1]/COLLISION_MASK_RES))]:
#             return False
#         else:
#             return True

# @jit(nopython=True)
# def index_to_coor(val):
#     return - MATRIX_RES*CELL_SIZE + CELL_SIZE*val

# # Converts coordinates to array index
# @jit(nopython=True)
# def coor_as_index(pos):
#     left = 0
#     right = MATRIX_RES*2 + 1
#     while left != right:
#         # print(left, right)
#         center_val = (index_to_coor(left) + index_to_coor(right))/2
#         if center_val <= pos[0]:
#             left = int(math.floor((left + right + 1)/2))
#         elif center_val >= pos[0]:
#             right = int(math.floor((left + right)/2))
#     index_i = left

#     left = 0
#     right = MATRIX_RES*2 + 1
#     while left != right:
#         center_val = (index_to_coor(left) + index_to_coor(right))/2
#         if center_val <= pos[1]:
#             left = int(math.floor((left + right + 1)/2))
#         elif center_val >= pos[1]:
#             right = int(math.floor((left + right)/2)) 
#     index_j = left

#     min_dif = 1000000
#     min_angle = 0
#     for i in range(ANGLE_RES):
#         if i == 0:
#             local_dif = min(pos[2], abs(2*math.pi - pos[2]))
#         else:
#             local_dif = abs(i*2*math.pi/ANGLE_RES - pos[2])
#         # print(i, local_dif, min_dif)
#         if local_dif < min_dif:
#             min_angle = i
#             min_dif = local_dif
#     return index_i, index_j, min_angle

obstacles = [(4, 10, 3), (12, 3, 3), (3, 10, 0), (5, 3, 3), (3, 17, 3)]
# collision_mask = get_collision_mask(obstacles)

# cost_grid = np.load('precomputed_costmatrices/cost_grid.npy')
# prev_grid = np.load('precomputed_costmatrices/prev_grid.npy')
# @jit(nopython=True)
# def constrained_dist(pos1, pos2):
#     relative_theta = (pos2[2] - pos1[2])%(2*math.pi)
#     fin_distance = distance(pos1, pos2)
#     relative_x = math.sin(relative_theta)*fin_distance
#     relative_y = math.cos(relative_theta)*fin_distance
#     idx_i, idx_j, idx_theta = coor_as_index((relative_x, relative_y, relative_theta))
#     return cost_grid[idx_i][idx_j][idx_theta]

# LEFT = 0
# STRAIGHT = 1
# RIGHT = 2

# TURNING_CIRCLE = 17

# @jit(nopython=True)
# def compute_pos(pos, direction, distance):
#     if direction == STRAIGHT:
#         new_pos_x = pos[0] + math.sin(pos[2]) * distance
#         new_pos_y = pos[1] + math.cos(pos[2]) * distance
#         return new_pos_x, new_pos_y, pos[2]
#     elif direction == LEFT:
#         turning_angle = distance/TURNING_CIRCLE
#         new_pos_x = pos[0] - TURNING_CIRCLE * (math.cos(pos[2]) - math.cos(turning_angle - pos[2]))
#         new_pos_y = pos[1] + TURNING_CIRCLE * (math.sin(pos[2]) + math.sin(turning_angle - pos[2]))
#         return new_pos_x, new_pos_y, pos[2] - turning_angle
#     elif direction == RIGHT:
#         turning_angle = distance/TURNING_CIRCLE
#         new_pos_x = pos[0] + TURNING_CIRCLE * (math.cos(pos[2]) - math.sin(math.pi/2 - pos[2] - turning_angle))
#         new_pos_y = pos[1] + TURNING_CIRCLE * (math.cos(math.pi/2 - pos[2] - turning_angle) - math.sin(pos[2]))
#         return new_pos_x, new_pos_y, pos[2] + turning_angle

# step = 2
# steering_mult = 0
# reverse_mult = 0.5
# angle_mult = 0.8
# distance_mult = 0.1
# goal_mult = 1
# precom_mult = 1
# # @jit(nopython=True)
# def next_valid_moves(pos, dist_traveled, reverses, waypoint_2, prev_path):
#     next_moves = []
#     # print(next_moves)
#     next_pos = compute_pos(pos, LEFT, step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] - step))*reverse_mult
#         prev_path.append((LEFT, step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] - step), next_pos, prev_path[:]))
#         prev_path.pop()

#     next_pos = compute_pos(pos, STRAIGHT, step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] - step))*reverse_mult
#         prev_path.append((STRAIGHT, step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] - step), next_pos, prev_path[:]))
#         prev_path.pop()

#     next_pos = compute_pos(pos, RIGHT, step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] - step))*reverse_mult
#         prev_path.append((RIGHT, step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] - step), next_pos, prev_path[:]))
#         prev_path.pop()

#     next_pos = compute_pos(pos, LEFT, -step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] + step))*reverse_mult
#         prev_path.append((LEFT, -step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] + step), next_pos, prev_path[:]))
#         prev_path.pop()

#     next_pos = compute_pos(pos, STRAIGHT, -step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] + step))*reverse_mult
#         prev_path.append((STRAIGHT, -step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] + step), next_pos, prev_path[:]))
#         prev_path.pop()

#     next_pos = compute_pos(pos, RIGHT, -step)
#     if is_valid_pos(next_pos):
#         cost = constrained_dist(pos, next_pos)*precom_mult + distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + (abs(next_pos[2] - waypoint_2[2])%(2*math.pi))*angle_mult
#         if len(prev_path) > 0:
#             cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + (reverses + abs(prev_path[len(prev_path) - 1][1] + step))*reverse_mult
#         prev_path.append((RIGHT, -step))
#         next_moves.append((cost, dist_traveled + step, reverses + abs(prev_path[len(prev_path) - 1][1] + step), next_pos, prev_path[:]))
#         prev_path.pop()
#     return next_moves

# def findpath(waypoint_1, waypoint_2):
#     x = []
#     y = []
#     p_queue = [(0, 0, 0, waypoint_1, [])]
#     heapq.heapify(p_queue)
#     it = 0
#     min_cost = 1000000000
#     min_dist = 0
#     min_angle = 0
#     while len(p_queue) > 0:

#         current = heapq.heappop(p_queue)
#         # print(it, len(p_queue))
#         cost = current[0]
#         dist_traveled = current[1]
#         reverses = current[2]
#         pos = current[3]
#         prev_path = current[4]

#         x.append(pos[0])
#         y.append(pos[1])
#         # if min_dist > distance(pos, waypoint_2):
#         #     min_dist = distance(pos, waypoint_2)
#         #     min_angle = abs(pos[2] - waypoint_2[2])
#         # min_cost = min(min_cost, cost)
#         # print(min_dist, min_angle)
#         if it % 200000 == 0 and it != 0:
#             # print(len(p_queue), len(x))
#             fig = plt.figure()
#             ax = plt.gca()
#             ax.set_aspect('equal', adjustable='box')
#             ax.set_xlim([0, 200])
#             ax.set_ylim([0, 200])
#             for i in range(collision_mask.shape[0]):
#                 for j in range(collision_mask.shape[0]):
#                     if collision_mask[i][j]:
#                         ax.add_patch(Rectangle((i*COLLISION_MASK_RES, j*COLLISION_MASK_RES), COLLISION_MASK_RES, COLLISION_MASK_RES, facecolor='gray'))
#             plt.scatter(x, y, s=1)
#             plt.show()



#             for obstacle in obstacles:
#                 ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))

#             x = []
#             y = []
#         # print(distance(pos, waypoint_2))
#         if distance(pos, waypoint_2) < 1 and abs(pos[2] - waypoint_2[2]) < math.pi/18:
#             path_x = []
#             path_y = []
#             cur_pos = waypoint_1
#             for operation in prev_path:
#                 cur_pos = compute_pos(cur_pos, operation[0], operation[1])
#                 path_x.append(cur_pos[0])
#                 path_y.append(cur_pos[1])
#             fig = plt.figure()
#             ax = plt.gca()
#             ax.set_aspect('equal', adjustable='box')
#             ax.set_xlim([0, 200])
#             ax.set_ylim([0, 200])
#             for i in range(collision_mask.shape[0]):
#                 for j in range(collision_mask.shape[0]):
#                     if collision_mask[i][j]:
#                         ax.add_patch(Rectangle((i*COLLISION_MASK_RES, j*COLLISION_MASK_RES), COLLISION_MASK_RES, COLLISION_MASK_RES, facecolor='gray'))
#             plt.plot(path_x, path_y)
#             plt.show()



#             for obstacle in obstacles:
#                 ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))
#             return prev_path
        
#         next_moves = next_valid_moves(pos, dist_traveled, reverses, waypoint_2, prev_path)
#         # print(len(p_queue))
#         for move in next_moves:
#             heapq.heappush(p_queue, move)
#         it += 1


#####

cost_grid = np.full((20, 20, 4), 10000000000, dtype=int)
prev_grid = np.zeros((20, 20, 4, 3), dtype=int)
coll_grid = np.zeros((20, 20), dtype=int)

# 0 = UP
# 1 = DOWN
# 2 = LEFT
# 3 = RIGHT

UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

# def read_steering_collision(filename):
#     file = open(filename, 'r')
#     lines = file.readlines()
#     mask_len = lines[0]
#     steering_mask_tmp = np.ones((4, 4, length, length), dtype=bool)
#     steering_mask_start_end = np.empty((4, 4, 2))
#     mask_idx = 0
#     orient1 = -1
#     orient2 = -1
#     is_title = True
#     for line_idx in range(lines):
#         if line_idx == 0:
#             continue
#         if is_title:
#             orient1 = lines[line_idx][0]
#             orient2 = lines[line_idx][1]
#             is_title = False
#             mask_idx = 0
#         else:
#             for i in range(mask_len):
#                 if lines[line_idx][i] == '1':
#                     steering_mask_start_end[]

def get_disc_collision_mask():
    for obstacle in obstacles:
        coll_grid[int(obstacle[0])][int(obstacle[1])] = 1
    return coll_grid

def create_graph():
    edges = np.zeros((20, 20, 4, 20, 20, 4), dtype=int)
    for idx_x in range(0, 20):
        for idx_y in range(0, 20):
            for idx_theta in range(0, 4):
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
            print("ok")
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
        # heapq.heappop(p_queue)
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
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim([0, 20])
    ax.set_ylim([0, 20])
    # for k in range(collision_mask.shape[0]):
    #     for l in range(collision_mask.shape[0]):
    #         if collision_mask[k][l]:
    #             ax.add_patch(Rectangle((k, l), 1, 1, facecolor='gray'))

    plt.plot(path_x, path_y)

    for obstacle in obstacles:
        ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1))
    # for point in prev_path:
    #     ax.annotate("", xy=(point[0] + math.sin(math.radians(point[2]))*2, point[1] + math.cos(math.radians(point[2]))*2), xytext=(point[0], point[1]),
    #     arrowprops=dict(arrowstyle="->"))

    plt.show()
print(instr)
print("Time taken: " + str(time.time() - start) + " sec")
# import socket

# HOST = '0.0.0.0' # Enter IP or Hostname of your server
# PORT = 8888 # Pick an open Port (1000+ recommended), must match the server port
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST,PORT))

# #Lets loop awaiting for your input
# while True:
#     command = raw_input('Enter your command: ')
#     s.send(command)
#     reply = s.recv(1024)
#     if reply == 'Terminate':
#         break

#     print(reply)
