import numpy as np
import heapq
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits import mplot3d

# Precomputes all the costs for different starting and ending positions (without obstacles)
# Ref paper: Practical Search Techniques in Path Planning for Autonomous Driving 
# Paper link: https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf

MATRIX_RES = 10
CELL_SIZE = 200/MATRIX_RES
ANGLE_RES = 16
# 10x10x4 pre-compute costs
cost_grid = np.full((MATRIX_RES*2 + 1, MATRIX_RES*2 + 1, ANGLE_RES), 10000000)
prev_grid = np.zeros((MATRIX_RES*2 + 1, MATRIX_RES*2 + 1, ANGLE_RES, 3))



MIN_TURNING_RADS = 17

MIN_COLLISION_DIST = 20

LEFT = 0
STRAIGHT = 1
RIGHT = 2


def is_valid_pos(pos):
    if pos[0] <= MIN_COLLISION_DIST or pos[1] <= MIN_COLLISION_DIST or \
        pos[0] > 200 - MIN_COLLISION_DIST or pos[1] > 200 - MIN_COLLISION_DIST:
        return False
    else:
        return True

def distance(pos_1, pos_2):
    return math.sqrt((pos_1[0] - pos_2[0])**2 + (pos_1[1] - pos_2[1])**2)

def compute_pos(pos, direction, distance):
    if direction == STRAIGHT:
        new_pos_x = pos[0] + math.sin(math.radians(pos[2])) * distance
        new_pos_y = pos[1] + math.cos(math.radians(pos[2])) * distance
        return new_pos_x, new_pos_y, pos[2]
    elif direction == LEFT:
        turning_angle = 360.0*distance/(2.0*math.pi*TURNING_RAD)
        new_pos_x = pos[0] - TURNING_RAD * (math.cos(math.radians(pos[2])) - math.cos(math.radians(turning_angle - pos[2])))
        new_pos_y = pos[1] + TURNING_RAD * (math.sin(math.radians(pos[2])) + math.sin(math.radians(turning_angle - pos[2])))
        return new_pos_x, new_pos_y, pos[2] - turning_angle
    elif direction == RIGHT:
        turning_angle = 360.0*distance/(2.0*math.pi*TURNING_RAD)
        new_pos_x = pos[0] + TURNING_RAD * (math.cos(math.radians(pos[2])) - math.sin(math.radians(90.0 - pos[2] - turning_angle)))
        new_pos_y = pos[1] + TURNING_RAD * (math.cos(math.radians(90.0 - pos[2] - turning_angle)) - math.sin(math.radians(pos[2])))
        return new_pos_x, new_pos_y, pos[2] + turning_angle
x = []
y = []

def index_to_coor(val):
    return - MATRIX_RES*CELL_SIZE + CELL_SIZE*val

# Converts coordinates to array index
def coor_as_index(pos):
    left = 0
    right = MATRIX_RES*2 + 1
    while left != right:
        # print(left, right)
        center_val = (index_to_coor(left) + index_to_coor(right))/2
        if center_val <= pos[0]:
            left = int(math.floor((left + right + 1)/2))
        elif center_val >= pos[0]:
            right = int(math.floor((left + right)/2))
    index_i = left

    left = 0
    right = MATRIX_RES*2 + 1
    while left != right:
        center_val = (index_to_coor(left) + index_to_coor(right))/2
        if center_val <= pos[1]:
            left = int(math.floor((left + right + 1)/2))
        elif center_val >= pos[1]:
            right = int(math.floor((left + right)/2)) 
    index_j = left

    min_dif = 1000000
    min_angle = 0
    for i in range(ANGLE_RES):
        if i == 0:
            local_dif = min(pos[2], abs(360 - pos[2]))
        else:
            local_dif = abs(i*360/ANGLE_RES - pos[2])
        # print(i, local_dif, min_dif)
        if local_dif < min_dif:
            min_angle = i
            min_dif = local_dif
    return index_i, index_j, min_angle
# import sys
# np.set_printoptions(threshold=sys.maxsize)
print(coor_as_index((0, 0, 0)))
# exit(0)
def compute_cost(): 
    x = np.linspace(-MATRIX_RES, MATRIX_RES, MATRIX_RES*2+1)
    y = np.linspace(-MATRIX_RES, MATRIX_RES, MATRIX_RES*2+1)
    val = np.zeros((MATRIX_RES*2+1, MATRIX_RES*2+1))

    p_queue = [(0, (0, 0, 0), (0, 0, 0))]
    heapq.heapify(p_queue)
    while len(p_queue) > 0:
        current = heapq.heappop(p_queue)
        cost = current[0]
        pos = current[1]
        prev = current[2]

        cur_index = coor_as_index(pos)
        prev_index = coor_as_index(prev)
        # print(cost, cur_index)
        if cost_grid[cur_index[0]][cur_index[1]][cur_index[2]] > cost:
            cost_grid[cur_index[0]][cur_index[1]][cur_index[2]] = cost
            for i in range(3):
                prev_grid[cur_index[0]][cur_index[1]][cur_index[2]][i] = prev_index[i]

            arc_dist = 0
            new_theta = pos[2]
            for i in range(-MATRIX_RES, MATRIX_RES+1):
                for j in range(-MATRIX_RES, MATRIX_RES+1):
                    delta_x = i - pos[0]
                    delta_y = j - pos[1]
                    if delta_x == 0:
                        arc_dist = abs(delta_y)
                        new_theta = pos[2]
                    else:
                        distance = math.sqrt(delta_x**2 + delta_y**2)
                        alpha_rad = math.atan(delta_y/delta_x)
                        turning_angle_rad = math.pi - 2*(alpha_rad + math.radians(pos[2]))
                        if math.sin(turning_angle_rad) == 0:
                            continue
                        else:
                            turning_radius = math.sin(alpha_rad + math.radians(pos[2]))*distance/math.sin(turning_angle_rad)

                        new_theta = math.degrees(math.pi - 2*alpha_rad - math.radians(pos[2]))
                        # if turning_radius > 17:
                        if delta_x >= 0:
                            if turning_angle_rad > math.pi:
                                arc_dist = (math.pi*2-turning_angle_rad)*turning_radius
                            elif turning_angle_rad < math.pi:
                                arc_dist = turning_angle_rad*turning_radius
                            else:
                                arc_dist = turning_angle_rad*delta_x/2

                        delta_x = pos[0] - i
                        delta_y = j - pos[1]
                        distance = math.sqrt(delta_x**2 + delta_y**2)
                        alpha_rad = math.atan(delta_y/delta_x)
                        turning_angle_rad = math.pi - 2*(alpha_rad + math.radians(pos[2]))
                        if math.sin(turning_angle_rad) == 0:
                            continue
                        else:
                            turning_radius = math.sin(alpha_rad + math.radians(pos[2]))*distance/math.sin(turning_angle_rad)

                        new_theta = math.degrees(math.pi*2 - (math.pi - 2*alpha_rad - math.radians(pos[2])))
                        if delta_x >= 0:
                            if turning_angle_rad > math.pi:
                                arc_dist = (math.pi*2-turning_angle_rad)*turning_radius
                            elif turning_angle_rad < math.pi:
                                arc_dist = turning_angle_rad*turning_radius
                            else:
                                arc_dist = turning_angle_rad*delta_x/2
                    next_index = coor_as_index((i, j, new_theta))
                    if cost_grid[next_index[0]][next_index[1]][next_index[2]] > cost + arc_dist:
                        print(arc_dist, (i, j, new_theta))
                        heapq.heappush(p_queue, (cost + arc_dist, (i, j, new_theta), pos))

                    # new_theta = 
                    # distance = math.sqrt((i - pos[0])**2 + (j - pos[1])**2)
    for i in range(MATRIX_RES*2+1):
        for j in range(MATRIX_RES*2+1):
            val[i][j] = cost_grid[i][j][0]
    print(val)
    X, Y = np.meshgrid(x, y)

    fig = plt.figure()
    ax = plt.gca()
    cm = plt.cm.get_cmap('viridis')
    ax.set_aspect('equal', adjustable='box')
    plt.scatter(X, Y, c=val, cmap=cm)
    # ax.plot_surface(X, Y, val)
    # plt.contourf(X, Y, val)
    plt.show()


compute_cost()



def pathfind(waypoint_1, waypoint_2):
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

        x.append(pos[0])
        y.append(pos[1])
        if it % 200000 == 0:
            print(len(p_queue), len(x))
            fig = plt.figure()
            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlim([0, 200])
            ax.set_ylim([0, 200])
            plt.scatter(x, y, s=1)
            plt.show()



        #     for obstacle in obstacles:
        #         ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 10, 10))

        #     x = []
        #     y = []


        # prev_path.append(pos)

        if distance(pos, waypoint_2) < 1 and abs(pos[2] - waypoint_2[2]) < 5:
            print(waypoint_1, waypoint_2)
            print(current)
            path_x = []
            path_y = []
            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlim([0, 200])
            ax.set_ylim([0, 200])

            cur_pos = waypoint_1
            for operation in prev_path:
                cur_pos = compute_pos(cur_pos, operation[0], operation[1])
                path_x.append(cur_pos[0])
                path_y.append(cur_pos[1])
            
            plt.plot(path_x, path_y)

            plt.show()
            return cost, prev_path[:]

        # cost = distance(pos, waypoint_2) + dist_traveled + 1 + abs(pos[2] - waypoint_2[2])*0.05

        # step = distance(pos, waypoint_2)/10
        step = 4
        steering_mult = 0.1
        reverse_mult = 3
        angle_mult = 0.5
        distance_mult = 0.3
        goal_mult = 0.4
        
        next_pos = compute_pos(pos, LEFT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((LEFT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()
        
        next_pos = compute_pos(pos, STRAIGHT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((STRAIGHT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, RIGHT, step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] - step)*reverse_mult
            prev_path.append((RIGHT, step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, LEFT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - LEFT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((LEFT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, STRAIGHT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - STRAIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((STRAIGHT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()

        next_pos = compute_pos(pos, RIGHT, -step)
        if is_valid_pos(next_pos):
            cost = distance(next_pos, waypoint_2)*goal_mult + (dist_traveled + step)*distance_mult + abs(next_pos[2] - waypoint_2[2])*angle_mult
            if len(prev_path) > 0:
                cost += abs(prev_path[len(prev_path) - 1][0] - RIGHT)*steering_mult + abs(prev_path[len(prev_path) - 1][1] + step)*reverse_mult
            prev_path.append((RIGHT, -step))
            heapq.heappush(p_queue, (cost, dist_traveled + step, next_pos, prev_path[:]))
            prev_path.pop()
        it += 1

# # Starting Positions
# for x_1 in range(MATRIX_RES):
#     for y_1 in range(MATRIX_RES):
#         for theta_1 in range(ANGLE_RES):
#             start_pos = ((x_1 + 0.5)*GRID_SIZE, (y_1 + 0.5)*GRID_SIZE, theta_1*90)
#             if not is_valid_pos(start_pos):
#                 print(start_pos)
#                 continue
#             # Ending Positions
#             for x_2 in range(MATRIX_RES):
#                 for y_2 in range(MATRIX_RES):
#                     for theta_2 in range(4):
#                         end_pos = ((x_2 + 0.5)*GRID_SIZE, (y_2 + 0.5)*GRID_SIZE, theta_2*90)
#                         if not is_valid_pos(end_pos):
#                             continue
#                         pathfind(start_pos, end_pos)
