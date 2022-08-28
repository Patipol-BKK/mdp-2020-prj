import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits import mplot3d

MATRIX_RES = 10
CELL_SIZE = 200/MATRIX_RES
ANGLE_RES = 16

OBJ_DISTANCE = 30

MIN_COLLISION_DIST = 15

COLLISION_MASK_RES = 2

NUM_OBJ = 5

def distance(pos_1, pos_2):
    return math.sqrt((pos_1[0] - pos_2[0])**2 + (pos_1[1] - pos_2[1])**2)

def get_collision_mask(obstacles):
    collision_mask = np.zeros((int(200/COLLISION_MASK_RES), int(200/COLLISION_MASK_RES)))
    for i in range(int(200/COLLISION_MASK_RES)):
        for j in range(int(200/COLLISION_MASK_RES)):
            pos_x = i*COLLISION_MASK_RES + COLLISION_MASK_RES/2
            pos_y = j*COLLISION_MASK_RES + COLLISION_MASK_RES/2
            for obstacle in obstacles:
                corner_topleft = obstacle[0], obstacle[1] + 10
                corner_topright = obstacle[0] + 10, obstacle[1] + 10
                corner_botleft = obstacle[0], obstacle[1]
                corner_botright = obstacle[0] + 10, obstacle[1]

                if pos_x >= corner_topleft[0] and pos_x < corner_topright[0]:
                    if pos_y - corner_topleft[1] <= MIN_COLLISION_DIST and corner_botleft[1] - pos_y <= MIN_COLLISION_DIST:
                        collision_mask[i][j] = 1
                        break
                elif pos_y >= corner_botleft[1] and pos_y < corner_topleft[1]:
                    if corner_topleft[0] - pos_x <= MIN_COLLISION_DIST and pos_x - corner_topright[0] <= MIN_COLLISION_DIST:
                        collision_mask[i][j] = 1
                        break
                elif distance((pos_x, pos_y), corner_topleft) <= MIN_COLLISION_DIST:
                    collision_mask[i][j] = 1
                    break
                elif distance((pos_x, pos_y), corner_topright) <= MIN_COLLISION_DIST:
                    collision_mask[i][j] = 1
                    break
                elif distance((pos_x, pos_y), corner_botleft) <= MIN_COLLISION_DIST:
                    collision_mask[i][j] = 1
                    break
                elif distance((pos_x, pos_y), corner_botright) <= MIN_COLLISION_DIST:
                    collision_mask[i][j] = 1
                    break
    return collision_mask

obstacles = [(40, 100, 3), (120, 30, 3), (30, 100, 0), (50, 30, 3), (30, 170, 3)]
collision_mask = get_collision_mask(obstacles)
