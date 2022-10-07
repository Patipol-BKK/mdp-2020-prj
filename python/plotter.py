import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import math
from utils import *
# import alphashape
# from descartes import PolygonPatch

TURNING_L = 2.2
TURNING_R = TURNING_L + 0.1

WIDTH = 2
LENGTH = 2.3
BACK_INSET = 0.3

UP = 0
DOWN = 1
LEFT = 2
RIGHT = 3

def point_is_collided(pos, cell_x, cell_y):
	if pos[0] >= cell_x and pos[0] <= cell_x + 1 and pos[1] >= cell_y and pos[1] <= cell_y + 1:
		return True
	return False

def turn_right(pos, alpha):
	x, y, theta = pos[0], pos[1], pos[2]
	x_turning_center = x + math.cos(math.radians(theta))*TURNING_R
	y_turning_center = y - math.sin(math.radians(theta))*TURNING_R

	x_new = x_turning_center + math.sin(math.radians(theta + alpha - 90))*TURNING_R
	y_new = y_turning_center + math.cos(math.radians(theta + alpha - 90))*TURNING_R
	theta_new = (theta + alpha)%360

	return x_new, y_new, theta_new

def turn_left(pos, alpha):
	x, y, theta = pos[0], pos[1], pos[2]
	x_turning_center = x - math.cos(math.radians(theta))*TURNING_L
	y_turning_center = y + math.sin(math.radians(theta))*TURNING_L

	x_new = x_turning_center + math.cos(math.radians(alpha - theta))*TURNING_L
	y_new = y_turning_center + math.sin(math.radians(alpha - theta))*TURNING_L
	theta_new = (theta - alpha)%360

	return x_new, y_new, theta_new

def move_straight(pos, distance):
	x, y, theta = pos[0], pos[1], pos[2]
	x_new = x + math.sin(math.radians(theta))*distance/10
	y_new = y + math.cos(math.radians(theta))*distance/10

	return x_new, y_new, theta

def distance_to_collision(pos, obstacles):
	cur_pos = pos
	collided = False
	distance = 0
	while not collided:
		x_front = cur_pos[0] + math.sin(math.radians(cur_pos[2]))*(LENGTH - BACK_INSET)
		y_front = cur_pos[1] + math.cos(math.radians(cur_pos[2]))*(LENGTH - BACK_INSET)
		for obstacle in obstacles:
			collided = point_is_collided((x_front, y_front), obstacle[0], obstacle[1])
			if collided:
				break
		if not collided:
			cur_pos = move_straight(cur_pos, 1)
			distance += 1
	return distance


def get_outer_points(pos):
	x, y, theta = pos[0], pos[1], pos[2]
	x_left_front = x + math.sin(math.radians(theta))*(LENGTH - BACK_INSET) - math.cos(math.radians(theta))*(WIDTH/2)
	y_left_front = y + math.cos(math.radians(theta))*(LENGTH - BACK_INSET) + math.sin(math.radians(theta))*(WIDTH/2)

	x_right_front = x + math.sin(math.radians(theta))*(LENGTH - BACK_INSET) + math.cos(math.radians(theta))*(WIDTH/2)
	y_right_front = y + math.cos(math.radians(theta))*(LENGTH - BACK_INSET) - math.sin(math.radians(theta))*(WIDTH/2)

	x_left_back = x - math.sin(math.radians(theta))*BACK_INSET - math.cos(math.radians(theta))*(WIDTH/2)
	y_left_back = y - math.cos(math.radians(theta))*BACK_INSET + math.sin(math.radians(theta))*(WIDTH/2)

	x_right_back = x - math.sin(math.radians(theta))*BACK_INSET + math.cos(math.radians(theta))*(WIDTH/2)
	y_right_back = y - math.cos(math.radians(theta))*BACK_INSET - math.sin(math.radians(theta))*(WIDTH/2)

	return [(x_left_front, y_left_front), (x_right_front, y_right_front), (x_right_back, y_right_back), (x_left_back, y_left_back)]

def execute_instr(pos, instr, obstacles):
	x, y, theta = pos[0], pos[1], pos[2]
	if instr[3] == 'F':
		if instr[4] == 'W':
			x_new, y_new, theta_new = move_straight(pos, int(instr[5:8]))
		elif instr[4] == 'L':
			x_new, y_new, theta_new = turn_left(pos, int(instr[5:8]))
		elif instr[4] == 'R':
			x_new, y_new, theta_new = turn_right(pos, int(instr[5:8]))
	elif instr[3] == 'B':
		if instr[4] == 'W':
			x_new, y_new, theta_new = move_straight(pos, -int(instr[5:8]))
		elif instr[4] == 'L':
			x_new, y_new, theta_new = turn_left(pos, -int(instr[5:8]))
		elif instr[4] == 'R':
			x_new, y_new, theta_new = turn_right(pos, -int(instr[5:8]))
	return x_new, y_new, theta_new

def plot_instr(init_pos, instr_list, obstacles):
	x = []
	y = []
	cur_pos = init_pos
	outer_pos = get_outer_points(cur_pos)
	x_car = []
	y_car = []
	for i in range(4):
		x_car.append(outer_pos[i][0])
		y_car.append(outer_pos[i][1])
	x_car.append(outer_pos[0][0])
	y_car.append(outer_pos[0][1])
	plt.plot(x_car, y_car, color='orange', alpha=0.5)
	plt.fill(x_car, y_car, color='orange', alpha=0.2)

	distances = np.zeros(10)

	for idx, instr in enumerate(instr_list):
		if instr[1] != 'S':
			continue
		if instr[3:5] == 'DM':
			index = int(instr[5])
			distances[index] = distance_to_collision(cur_pos, obstacles) - int(instr[6:])
			step_instr = 'PS|FW001'
			steps = distances[index]
		elif instr[3:5] == 'DR':
			step_instr = 'PS|FW001'
			steps = distances[index]
		else:
			steps = int(instr[5:8])
			step_instr = instr[:5] + '001'
		for i in range(int(steps)):
			cur_pos = execute_instr(cur_pos, step_instr, obstacles)
			x.append(cur_pos[0])
			y.append(cur_pos[1])


		x_car = []
		y_car = []
		outer_pos = get_outer_points(cur_pos)
		for i in range(4):
			x_car.append(outer_pos[i][0])
			y_car.append(outer_pos[i][1])
		x_car.append(outer_pos[0][0])
		y_car.append(outer_pos[0][1])
		plt.plot(x_car, y_car, color='orange', alpha=0.5)
		plt.fill(x_car, y_car, color='orange', alpha=0.2)
		# plt.
	plt.plot(x, y)

	return cur_pos


	

def plot_obstacles(obstacles):
	ax = plt.gca()
	for obstacle in obstacles:
		ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1,color='gray'))
		if obstacle[2] == 0:
			plt.plot([obstacle[0],obstacle[0]+1],[obstacle[1]+1,obstacle[1]+1],color='red')
		elif obstacle[2] == 1:
			plt.plot([obstacle[0],obstacle[0]+1],[obstacle[1],obstacle[1]],color='red')
		elif obstacle[2] == 2:
			plt.plot([obstacle[0],obstacle[0]],[obstacle[1]+1,obstacle[1]],color='red')
		elif obstacle[2] == 3:
			plt.plot([obstacle[0]+1,obstacle[0]+1],[obstacle[1]+1,obstacle[1]],color='red')

def midpoint(p1, p2):
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2

def is_collided(x, y, pos):
	outer_points = get_outer_points(pos)

	collision_test_points = outer_points
	collision_test_points.append(midpoint(outer_points[0], outer_points[1]))
	collision_test_points.append(midpoint(outer_points[1], outer_points[2]))
	collision_test_points.append(midpoint(outer_points[2], outer_points[3]))
	collision_test_points.append(midpoint(outer_points[3], outer_points[0]))
	collision_test_points.append(midpoint(outer_points[0], outer_points[2]))
	for point in collision_test_points:
		if point_is_collided(point, x, y):
			return True
	return False

def get_collided_cells(init_pos, instr_list, obstacles, x_lim, y_lim):
	cur_pos = init_pos
	collided_cells = np.zeros((x_lim, y_lim))
	distances = np.zeros(10)
	for idx, instr in enumerate(instr_list):
		if instr[1] != 'S':
			continue
		if instr[3:5] == 'DM':
			index = int(instr[5])
			distances[index] = distance_to_collision(cur_pos, obstacles) - int(instr[6:])
			step_instr = 'PS|FW001'
			steps = distances[index]
		elif instr[3:5] == 'DR':
			step_instr = 'PS|FW001'
			steps = distances[index]
		else:
			steps = int(instr[5:8])
			step_instr = instr[:5] + '001'
		for i in range(int(steps)):
			# print(step_instr)
			cur_pos = execute_instr(cur_pos, step_instr, obstacles)
			for idx_x in range(x_lim):
				for idx_y in range(y_lim):
					if is_collided(idx_x, idx_y, cur_pos):
						collided_cells[idx_x][idx_y] = 1
	print("Final coor: ", cur_pos)
	return collided_cells

def update_collided_cells(pos, collided_cells, x_lim, y_lim):
	for idx_x in range(x_lim):
		for idx_y in range(y_lim):
			if is_collided(idx_x, idx_y, pos):
				collided_cells[idx_x][idx_y] = 1
	return collided_cells

def plot_collided_cells(collided_cells, x_lim, y_lim):
	ax = plt.gca()
	for idx_x in range(x_lim):
		for idx_y in range(y_lim):
			if collided_cells[idx_x][idx_y]:
				ax.add_patch(Rectangle((idx_x, idx_y), 1, 1,color='blue',alpha=0.06))


# obstacles = [(4, 10, 3, 4), (12, 3, 3, 6), (3, 10, 0, 5), (5, 3, 3, 3), (3, 17, 3, 2)] 
# # instr_list = ['PR|start', 'PR|O3', 'PS|FW030', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW020', 'PS|FL090', 'PS|BW020', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S3', 'PR|O6', 'PS|FW023', 'PS|BR090', 'PS|FW003', 'PS|BL090', 'PS|BW010', 'PS|BL090', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S6', 'PR|O4', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|FW040', 'PR|S4', 'PR|O2', 'PS|BW013', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|BW020', 'PR|S2', 'PR|O5', 'PS|FL180', 'PS|BW030', 'PS|BL090', 'PR|S5']

# collided_cells = get_collided_cells((2.5, 0.5, 0), instr_list)

# plot_collided_cells(collided_cells)
# plot_instr((2.5, 0.5, 0), instr_list)
# plot_obstacle(obstacles)
# ax = plt.gca()
# ax.set_xticks(np.arange(0,21,1))
# ax.set_yticks(np.arange(0,21,1))
# ax.grid(which='major', alpha=0.2)
# plt.show()

