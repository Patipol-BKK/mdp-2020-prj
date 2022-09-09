from dataclasses import dataclass
import math
import numpy as np
import pygame
import heapq
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# @dataclass
# class SceneObject:

WIDTH = 11
LENGTH = 23

LINK_X = 0.5    
LINK_Y = 3
LINK_GAP = 9

LINK_LENGTH = math.sqrt(LINK_Y**2 + LINK_X**2)
LINK_ANGLE = math.atan(LINK_X/LINK_Y)

WHEEL_WIDTH = 2.7
WHEEL_DIA = 6

PIVOT_GAP = LINK_GAP + 2*LINK_X
PIVOT_OFFSET = LINK_X + 3.8 - WHEEL_WIDTH/2

FRONT_BACK_GAP = 14.5

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

SCALE = 10
    
TURNING_CIRCLE = 15

OBJ_DISTANCE = 30

MIN_COLLISION_DIST = 15

COLLISION_MASK_RES = 2

NUM_OBJ = 2

START_POS = (170, 170, 0)

class __SteeringConverter:
    """
    Helper class to calculate steering angles and front wheel positions
    """
    @staticmethod
    def rightToLeft(link_length, link_angle, link_gap, wheel_angle):

        link_gap_l = link_gap + 2*math.sin(link_angle)*link_length

        internal_right_angle = math.pi/2 - link_angle - wheel_angle

        opp_length = math.sqrt(link_length**2 + link_gap_l**2 \
                        - 2*link_length*link_gap_l*math.cos(internal_right_angle))

        internal_left_angle = math.asin(link_length*math.sin(internal_right_angle)/opp_length) \
                        + math.acos((opp_length**2 + link_length**2 - link_gap**2)/(2*link_length*opp_length))

        left_angle = internal_left_angle - math.pi/2 + link_angle
        return left_angle

    @staticmethod
    def frontContactGap(left_angle, right_angle, pivot_gap, contact_offset):
        gap_x = math.cos(left_angle)*contact_offset + pivot_gap + math.cos(right_angle)*contact_offset
        gap_y = math.sin(left_angle)*contact_offset + math.sin(right_angle)*contact_offset
        gap = math.sqrt(gap_x**2 + gap_y**2)
        return gap

    @staticmethod
    def frontRightToBackRightGap(right_angle, contact_offset, front_back_gap):
        gap_x = contact_offset - math.cos(right_angle)*contact_offset
        gap_y = front_back_gap - math.sin(right_angle)*contact_offset
        gap = math.sqrt(gap_x**2 + gap_y**2)
        print(gap_x, gap_y)
        return gap

    @staticmethod
    def frontLeftToBackLeftGap(left_angle, contact_offset, front_back_gap):
        gap_x = contact_offset - math.cos(left_angle)*contact_offset
        gap_y = front_back_gap + math.sin(left_angle)*contact_offset
        gap = math.sqrt(gap_x**2 + gap_y**2)
        return gap_x, gap_y

class PositionVector:
    pos = np.zeros(4)
    vel = np.zeros(3)
    acc = np.zeros(3)

right_angle = 30
left_angle = math.degrees(__SteeringConverter.rightToLeft(LINK_LENGTH, LINK_ANGLE, LINK_GAP, math.radians(right_angle)))
print(round(left_angle, 3))
# print(__SteeringConverter.frontContactGap(math.radians(left_angle), math.radians(right_angle), PIVOT_GAP, PIVOT_OFFSET))
# print(__SteeringConverter.frontRightToBackRightGap(math.radians(right_angle), PIVOT_OFFSET, FRONT_BACK_GAP))


for i in range (1, 91):
    right_angle = i
    left_angle = math.degrees(__SteeringConverter.rightToLeft(LINK_LENGTH, LINK_ANGLE, LINK_GAP, math.radians(right_angle)))
    val = __SteeringConverter.frontLeftToBackLeftGap(math.radians(left_angle), PIVOT_OFFSET, FRONT_BACK_GAP)
    print(i, math.sin(math.radians(90 - left_angle))*val[1]/math.sin(math.radians(left_angle)) + val[0] - 11/2 - 3.8 + 2.7/2)

UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

class Arena:
    # grid =   np.asarray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    obstacles = []
    def add_obstacle(self, x, y, orientation):
        self.grid[x][19 - y] = orientation
        self.obstacles.append((x, y, orientation))

obstacles = [(40, 60, UP),
             (100, 40, UP),
             (10, 80, DOWN),
             (40, 140, RIGHT),
             (180, 60, UP),
             (70, 60, DOWN),
             (30, 100, UP),]
# def generate_path(x, y, angle, x_obj, y_obj, angle):

routes = []
def add_route(route):
    if len(route) >= NUM_OBJ:
        routes.append(route[:])
    for i in range(NUM_OBJ):
        if not i in route:
            route.append(i)
            # print(route)
            add_route(route)
            route.pop()

def get_waypoint(obstacle):
    obstacle = obstacle[0] + 5, obstacle[1] + 5, obstacle[2]
    # obstacle[1] += 5
    # obstacle[2] += 5
    if obstacle[2] == UP:
        return obstacle[0], obstacle[1] - OBJ_DISTANCE, 180
    elif obstacle[2] == DOWN:
        return obstacle[0], obstacle[1] + OBJ_DISTANCE, 0
    elif obstacle[2] == LEFT:
        return obstacle[0] - OBJ_DISTANCE, obstacle[1], 90
    else:
        return obstacle[0] + OBJ_DISTANCE, obstacle[1], 270

add_route([])

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

def generate_obstacles(num_obstacles):
    gen_obstacles = []
    while len(gen_obstacles) < num_obstacles:
        collision_mask = get_collision_mask(gen_obstacles)
        collide = True
        
        obj_x = random.randint(0, 19) * 10
        obj_y = random.randint(0, 19) * 10
        obj_orient = random.randint(0, 3)

        obj_waypoint = get_waypoint((obj_x, obj_y, obj_orient))
        if obj_waypoint[0] <= MIN_COLLISION_DIST or obj_waypoint[1] <= MIN_COLLISION_DIST or \
            obj_waypoint[0] > 200 - MIN_COLLISION_DIST or obj_waypoint[1] > 200 - MIN_COLLISION_DIST:
            collide = True
        else:
            prev_collide = False
            for obstacle in gen_obstacles:
                obst_waypoint = get_waypoint(obstacle)
                if collision_mask[int(math.floor(obst_waypoint[0]/COLLISION_MASK_RES))][int(math.floor(obst_waypoint[1]/COLLISION_MASK_RES))]:
                    prev_collide = True

            if not collision_mask[int(math.floor(obj_waypoint[0]/COLLISION_MASK_RES))][int(math.floor(obj_waypoint[1]/COLLISION_MASK_RES))] and not prev_collide:
                collide = False
            
        if collide:
            gen_obstacles = []
        else:
            gen_obstacles.append((obj_x, obj_y, obj_orient))
    return gen_obstacles
# def generate_path():


# p_queue_routes = [(0, (0, 0, 90), [], route) for route in p_queue_routes]
# print(p_queue_routes)


 

def compute_pos(pos, direction, distance):
    if direction == STRAIGHT:
        new_pos_x = pos[0] + math.sin(math.radians(pos[2])) * distance
        new_pos_y = pos[1] + math.cos(math.radians(pos[2])) * distance
        return new_pos_x, new_pos_y, pos[2]
    elif direction == LEFT:
        turning_angle = 360.0*distance/(2.0*math.pi*TURNING_CIRCLE)
        new_pos_x = pos[0] - TURNING_CIRCLE * (math.cos(math.radians(pos[2])) - math.cos(math.radians(turning_angle - pos[2])))
        new_pos_y = pos[1] + TURNING_CIRCLE * (math.sin(math.radians(pos[2])) + math.sin(math.radians(turning_angle - pos[2])))
        return new_pos_x, new_pos_y, pos[2] - turning_angle
    elif direction == RIGHT:
        turning_angle = 360.0*distance/(2.0*math.pi*TURNING_CIRCLE)
        new_pos_x = pos[0] + TURNING_CIRCLE * (math.cos(math.radians(pos[2])) - math.sin(math.radians(90.0 - pos[2] - turning_angle)))
        new_pos_y = pos[1] + TURNING_CIRCLE * (math.cos(math.radians(90.0 - pos[2] - turning_angle)) - math.sin(math.radians(pos[2])))
        return new_pos_x, new_pos_y, pos[2] + turning_angle
# x = []
# y = []
# for i in range(20):
#     n_pos = compute_pos((0, 0, 10), RIGHT, i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])
#     n_pos = compute_pos((0, 0, 10), STRAIGHT, i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])
#     n_pos = compute_pos((0, 0, 10), LEFT, i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])

#     n_pos = compute_pos((0, 0, 10), RIGHT, -i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])
#     n_pos = compute_pos((0, 0, 10), STRAIGHT, -i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])
#     n_pos = compute_pos((0, 0, 10), LEFT, -i)
#     x.append(n_pos[0])
#     y.append(n_pos[1])

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.set_aspect('equal', adjustable='box')
# plt.scatter(x, y)
# plt.show()
# print(compute_pos((0, 0, 45), RIGHT, -47.124))
# obstacles = generate_obstacles(NUM_OBJ)
# obstacles = [(40, 100, 3), (120, 30, 3), (30, 100, 0), (50, 30, 3), (30, 170, 3)]
obstacles = [(130, 50, 2), (30, 150, 1)]
print(obstacles)
collision_mask = get_collision_mask(obstacles)

def is_valid_pos(pos):
    if pos[0] <= MIN_COLLISION_DIST or pos[1] <= MIN_COLLISION_DIST or \
        pos[0] > 200 - MIN_COLLISION_DIST or pos[1] > 200 - MIN_COLLISION_DIST:
        return False
    else:
        if collision_mask[int(math.floor(pos[0]/COLLISION_MASK_RES))][int(math.floor(pos[1]/COLLISION_MASK_RES))]:
            return False
        else:
            return True

path_x = []
path_y = []


def pathfind(waypoint_1, waypoint_2):
    x = []
    y = []
    p_queue = [(0, 0, waypoint_1, [])]
    heapq.heapify(p_queue)
    it = 0
    while len(p_queue) > 0:
        # print(p_queue)
        current = heapq.heappop(p_queue)
        cost = current[0]
        dist_traveled = current[1]
        pos = current[2]
        prev_path = current[3]

        # x.append(pos[0])
        # y.append(pos[1])
        # if it % 20000 == 0:
        #     print(len(p_queue), len(x))
        #     fig = plt.figure()
        #     ax = plt.gca()
        #     ax.set_aspect('equal', adjustable='box')
        #     ax.set_xlim([0, 200])
        #     ax.set_ylim([0, 200])
        #     for i in range(collision_mask.shape[0]):
        #         for j in range(collision_mask.shape[0]):
        #             if collision_mask[i][j]:
        #                 ax.add_patch(Rectangle((i*COLLISION_MASK_RES, j*COLLISION_MASK_RES), COLLISION_MASK_RES, COLLISION_MASK_RES, facecolor='gray'))
        #     plt.scatter(x, y, s=1)
        #     plt.show()



        #     for obstacle in obstacles:
        #         ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))

        #     x = []
        #     y = []


        # prev_path.append(pos)

        if distance(pos, waypoint_2) < 1 and abs(pos[2] - waypoint_2[2]) < 5:

            print(current)
            # path_x = []
            # path_y = []
            # ax = plt.gca()
            # ax.set_aspect('equal', adjustable='box')
            # ax.set_xlim([0, 200])
            # ax.set_ylim([0, 200])
            # for i in range(collision_mask.shape[0]):
            #     for j in range(collision_mask.shape[0]):
            #         if collision_mask[i][j]:
            #             ax.add_patch(Rectangle((i*COLLISION_MASK_RES, j*COLLISION_MASK_RES), COLLISION_MASK_RES, COLLISION_MASK_RES, facecolor='gray'))

            # cur_pos = waypoint_1
            # for operation in prev_path:
            #     cur_pos = compute_pos(cur_pos, operation[0], operation[1])
            #     path_x.append(cur_pos[0])
            #     path_y.append(cur_pos[1])
            
            # plt.plot(path_x, path_y)

            # for obstacle in obstacles:
            #     ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))
            # for point in prev_path:
            #     ax.annotate("", xy=(point[0] + math.sin(math.radians(point[2]))*2, point[1] + math.cos(math.radians(point[2]))*2), xytext=(point[0], point[1]),
            #     arrowprops=dict(arrowstyle="->"))

            plt.show()
            return cost, prev_path[:]

        # cost = distance(pos, waypoint_2) + dist_traveled + 1 + abs(pos[2] - waypoint_2[2])*0.05

        # step = distance(pos, waypoint_2)/10
        step = 4
        steering_mult = 1
        reverse_mult = 2
        angle_mult = 0.1
        distance_mult = 0.25
        
        next_pos = compute_pos(pos, LEFT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((LEFT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()
        
        next_pos = compute_pos(pos, STRAIGHT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((STRAIGHT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, RIGHT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((RIGHT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, LEFT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((LEFT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, STRAIGHT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((STRAIGHT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, RIGHT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2) + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((RIGHT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()
        it += 1

cost_mat = np.zeros((NUM_OBJ + 1, NUM_OBJ + 1))
path_mat = []
for i in range(NUM_OBJ + 1):
    path_list = []
    for j in range(NUM_OBJ + 1):
        path_list.append([])
    path_mat.append(path_list[:])

for i in range(NUM_OBJ):
    cost_mat[0][i + 1], path_mat[0][i + 1] = pathfind(START_POS, get_waypoint(obstacles[i]))
    for j in range(NUM_OBJ):
        cost_mat[i + 1][j + 1], path_mat[i + 1][j + 1] = pathfind(get_waypoint(obstacles[i]), get_waypoint(obstacles[j]))

add_route([])
min_cost = 1000000000
min_path = []
for route in routes:
    local_cost = cost_mat[0][route[0] + 1]
    local_path = path_mat[0][route[0] + 1][:]
    for i in range(len(route) - 1):
        local_cost += cost_mat[int(route[i] + 1)][int(route[i+1] + 1)]
        local_path += path_mat[int(route[i] + 1)][int(route[i+1] + 1)][:]
    if min_cost > local_cost:
        min_cost = local_cost
        min_path = local_path
print(min_cost)

path_x = []
path_y = []
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
ax.set_xlim([0, 200])
ax.set_ylim([0, 200])
for i in range(collision_mask.shape[0]):
    for j in range(collision_mask.shape[0]):
        if collision_mask[i][j]:
            ax.add_patch(Rectangle((i*COLLISION_MASK_RES, j*COLLISION_MASK_RES), COLLISION_MASK_RES, COLLISION_MASK_RES, facecolor='gray'))
cur_pos = START_POS
for operation in min_path:
    cur_pos = compute_pos(cur_pos, operation[0], operation[1])
    path_x.append(cur_pos[0])
    path_y.append(cur_pos[1])

plt.plot(path_x, path_y)

for obstacle in obstacles:
    ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))
# for point in prev_path:
#     ax.annotate("", xy=(point[0] + math.sin(math.radians(point[2]))*2, point[1] + math.cos(math.radians(point[2]))*2), xytext=(point[0], point[1]),
#     arrowprops=dict(arrowstyle="->"))

plt.show()




RENDERING_SIZE = 4
# pathfind()

# class Renderer
size = [800, 800]
screen = pygame.display.set_mode(size)

#Loop until the user clicks the close button.
done = False
clock = pygame.time.Clock()

screen.fill(WHITE)

pos_vec = np.asarray([0, 0, 0, 0])

robot_pos = (100, 100, 70)

while not done:
    for i in range(collision_mask.shape[0]):
        for j in range(collision_mask.shape[0]):
            if collision_mask[i][j]:
                pygame.draw.rect(screen, (255, 90, 90), (i*COLLISION_MASK_RES*RENDERING_SIZE, j*COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE))
                pygame.draw.rect(screen, (227, 57, 57), (i*COLLISION_MASK_RES*RENDERING_SIZE, j*COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE), 1)
            else:
                pygame.draw.rect(screen, (200, 200, 200), (i*COLLISION_MASK_RES*RENDERING_SIZE, j*COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE, COLLISION_MASK_RES*RENDERING_SIZE), 1)
    for obstacle in obstacles:
        pygame.draw.rect(screen, (100, 100, 100), (obstacle[0]*RENDERING_SIZE, obstacle[1]*RENDERING_SIZE, 10*RENDERING_SIZE, 10*RENDERING_SIZE))
    for i in range(20):
        for j in range(20):
            pygame.draw.rect(screen, (170, 170, 170), (i*10*RENDERING_SIZE, j*10*RENDERING_SIZE, 10*RENDERING_SIZE, 10*RENDERING_SIZE), 1)
    
    # This limits the while loop to a max of 10 times per second.
    # Leave this out and we will use all CPU we can.
    # clock.tick(10)

    for obstacle in obstacles:
        # pygame.draw.rect(screen, (100, 100, 100), (obstacle[0]*RENDERING_SIZE, obstacle[1]*RENDERING_SIZE, 10*RENDERING_SIZE, 10*RENDERING_SIZE))
        waypoint = get_waypoint(obstacle)
        # print(waypoint)
        pygame.draw.polygon(screen, color=(99,230,18), 
            points=[((waypoint[0]+3*math.sin(math.radians(waypoint[2])))*RENDERING_SIZE,(waypoint[1]-3*math.cos(math.radians(waypoint[2])))*RENDERING_SIZE),
            ((waypoint[0]+3*math.sin(math.radians(waypoint[2]+120)))*RENDERING_SIZE,(waypoint[1]-3*math.cos(math.radians(waypoint[2]+120)))*RENDERING_SIZE),
            ((waypoint[0]+3*math.sin(math.radians(waypoint[2]+240)))*RENDERING_SIZE,(waypoint[1]-3*math.cos(math.radians(waypoint[2]+240)))*RENDERING_SIZE)])
    
    # Robot Render
    pygame.draw.polygon(screen, color=(0, 100, 255), 
            points=[((robot_pos[0] - 11/2)*RENDERING_SIZE, (robot_pos[1] - 3)*RENDERING_SIZE),
                    ((robot_pos[0] + 11/2)*RENDERING_SIZE, (robot_pos[1] - 3)*RENDERING_SIZE),
                    ((robot_pos[0] + 11/2)*RENDERING_SIZE, (robot_pos[1] - 3 + 23)*RENDERING_SIZE),
                    ((robot_pos[0] - 11/2)*RENDERING_SIZE, (robot_pos[1] - 3 + 23)*RENDERING_SIZE)])
    # pygame.draw.rect(screen, (0, 100, 255), (robot_pos[0]*RENDERING_SIZE, robot_pos[1]*RENDERING_SIZE, WIDTH*RENDERING_SIZE, LENGTH*RENDERING_SIZE), 3)
    pygame.draw.line(screen, (0, 100, 255), 
            ((robot_pos[0] - 18.5/2 - math.cos(math.radians(left_angle))*PIVOT_OFFSET - math.sin(math.radians(left_angle))*WHEEL_DIA/2)*RENDERING_SIZE,
            (robot_pos[1] + FRONT_BACK_GAP + math.sin(math.radians(left_angle))*PIVOT_OFFSET - math.sin(math.radians(left_angle))*WHEEL_DIA/2)*RENDERING_SIZE),
            ((robot_pos[0] - 18.5/2 - math.cos(math.radians(left_angle))*PIVOT_OFFSET + math.sin(math.radians(left_angle))*WHEEL_DIA/2)*RENDERING_SIZE,
            (robot_pos[1] + FRONT_BACK_GAP + math.sin(math.radians(left_angle))*PIVOT_OFFSET + math.sin(math.radians(left_angle))*WHEEL_DIA/2)*RENDERING_SIZE))

    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop

    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
 
# Be IDLE friendly


pygame.quit()

