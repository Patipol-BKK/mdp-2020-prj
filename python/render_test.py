from plotter import *
from path_planner import *

def pos_to_cell(pos):
	if round(pos[2]/90) == 0:
		orient = UP
	elif round(pos[2]/90) == 2:
		orient = DOWN
	elif round(pos[2]/90) == 3:
		orient = LEFT
	else:
		orient = RIGHT

	return round(pos[0]-10.5), round(pos[1]-10.5), orient


# obstacles = [(-1, 7, -1), (-1, 8, -1), (-1, 9, -1), (-1, 10, -1), (-1, 11, -1), (-1, 12, -1), (-1, 13, -1), \
# 			 (0, 7, -1), (1, 7, -1), (2, 7, -1), (3, 7, -1), (4, 7, -1), \
# 			 (0, 13, -1), (1, 13, -1), (2, 13, -1), (3, 13, -1), (4, 13, -1), \
# 			 (11, 10, 2), \
# 			 # (18, 6, -1), (18, 7, -1), (18, 8, -1), (18, 9, -1), (18, 10, 2), (18, 11, -1), (18, 12, -1), (18, 13, -1), (18, 14, -1), \
# 			 (26, 6, -1), (26, 7, -1), (26, 8, -1), (26, 9, -1), (26, 10, 2), (26, 11, -1), (26, 12, -1), (26, 13, -1), (26, 14, -1)] 

# instr_list = ['PS|FW''PS|DM120', 'PS|FL060', 'PS|FR060', 'PS|DM225', 'PS|FR023', 'PS|FR067', 'PS|FW045', 'PS|FL090', \
# 			  'PS|FW025', 'PS|FL090', 'PS|FW090', 'PS|FL090', 'PS|DR200', 'PS|FW070', 'PS|FL090', 'PS|FW020', 'PS|FR090', 'PS|DR100']
# init_pos = (0.5, 10.5, 90)


# obstacles = [(0, 13, 3, 18), (3, 9, 1, 19), (5, 7, 2, 20), (18, 6, 1, 21), (17, 6, 0, 22), (17, 13, 1, 23), (14, 18, 3, 24)]
# instr_list = ['PR|start', 'PR|O20', 'PS|FW010', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW002', 'PS|BL090', 'PS|FW005', 'PS|BR090', 'PS|FW013', 'PR|S20', 'PR|O19', 'PS|BW013', 'PS|FR090', 'PS|BW005', 'PS|FL090', 'PS|BW034', 'PS|FL090', 'PS|BW002', 'PR|S19', 'PR|O21', 'PS|FW013', 'PS|BR090', 'PS|FW003', 'PS|BW110', 'PS|FW002', 'PS|BL090', 'PS|FW002', 'PR|S21', 'PR|O23', 'PS|BW002', 'PS|FL090', 'PS|BW005', 'PS|FR090', 'PS|BW006', 'PS|FR090', 'PS|BW015', 'PS|FL090', 'PS|BW012', 'PR|S23', 'PR|O22', 'PS|BW012', 'PS|FL090', 'PS|BW002', 'PS|FW003', 'PS|BR090', 'PS|FW023', 'PR|S22', 'PR|O18', 'PS|BW003', 'PS|FR090', 'PS|BW006', 'PS|FR090', 'PS|BW003', 'PS|FW010', 'PS|BW002', 'PS|FL090', 'PS|BW002', 'PS|FW070', 'PR|S18', 'PR|O24', 'PS|BW060', 'PS|FW003', 'PS|BR090', 'PS|FW025', 'PS|BL090', 'PS|FW005', 'PS|BR090', 'PS|FW015', 'PS|BL090', 'PS|FW002', 'PR|S24', 'PR|S0']
# init_pos = (2.5, 0.5, 0)

# # obstacles = [(4, 10, 3, 4), (12, 3, 3, 6), (3, 10, 0, 5), (5, 3, 3, 3), (3, 17, 3, 2)] 
# # instr_list = ['PR|start', 'PR|O3', 'PS|FW030', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW020', 'PS|FL090', 'PS|BW020', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S3', 'PR|O6', 'PS|FW023', 'PS|BR090', 'PS|FW003', 'PS|BL090', 'PS|BW010', 'PS|BL090', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S6', 'PR|O4', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|FW040', 'PR|S4', 'PR|O2', 'PS|BW013', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|BW020', 'PR|S2', 'PR|O5', 'PS|FL180', 'PS|BW030', 'PS|BL090', 'PR|S5']

# collided_cells = get_collided_cells((2.5, 0.5, 0), instr_list, obstacles, 20, 20)

# plot_collided_cells(collided_cells, 20, 20)
# plot_instr((2.5, 0.5, 0), instr_list, obstacles)
# plot_obstacles(obstacles)
# setup_arena_plot()
# plt.show()
def plot_collision_box(pos):
	x_car = []
	y_car = []
	outer_pos = get_outer_points(pos)
	for i in range(4):
		x_car.append(outer_pos[i][0])
		y_car.append(outer_pos[i][1])
	x_car.append(outer_pos[0][0])
	y_car.append(outer_pos[0][1])
	plt.plot(x_car, y_car, color='orange', alpha=0.5)
	plt.fill(x_car, y_car, color='orange', alpha=0.2)

def render_instr(car, instr, obstacles, frame, collided_cells, x_list, y_list):
	pos = car.pos
	if instr[1] != 'S':
		return frame, collided_cells, x_list, y_list
	if instr[3] == 'F' or instr[3] == 'B':
		if instr[4] == 'L':
			while car.servo_angle > -30:
				car.set_servo(car.servo_angle - 2)

				plt.clf()
				collided_cells = update_collided_cells(car.pos, collided_cells, 20, 20)
				plot_collided_cells(collided_cells, 20, 20)
				plt.plot(x_list, y_list)
				plot_collision_box(car.pos)
				car.plot_car()
				plot_obstacles(obstacles)
				setup_arena_plot()
				plt.savefig('pngs/' + str(frame) + '.png')
				frame += 1

		if instr[4] == 'R':
			while car.servo_angle < 30:
				car.set_servo(car.servo_angle + 2)

				plt.clf()
				collided_cells = update_collided_cells(car.pos, collided_cells, 20, 20)
				plot_collided_cells(collided_cells, 20, 20)
				plt.plot(x_list, y_list)
				plot_collision_box(car.pos)
				car.plot_car()
				plot_obstacles(obstacles)
				setup_arena_plot()
				plt.savefig('pngs/' + str(frame) + '.png')
				frame += 1

		if instr[4] == 'W':
			while car.servo_angle < 0:
				car.set_servo(car.servo_angle + 2)

				plt.clf()
				collided_cells = update_collided_cells(car.pos, collided_cells, 20, 20)
				plot_collided_cells(collided_cells, 20, 20)
				plt.plot(x_list, y_list)
				plot_collision_box(car.pos)
				car.plot_car()
				plot_obstacles(obstacles)
				setup_arena_plot()
				plt.savefig('pngs/' + str(frame) + '.png')
				frame += 1

			while car.servo_angle > 0:
				car.set_servo(car.servo_angle - 2)

				plt.clf()
				collided_cells = update_collided_cells(car.pos, collided_cells, 20, 20)
				plot_collided_cells(collided_cells, 20, 20)
				plt.plot(x_list, y_list)
				plot_collision_box(car.pos)
				car.plot_car()
				plot_obstacles(obstacles)
				setup_arena_plot()
				plt.savefig('pngs/' + str(frame) + '.png')
				frame += 1

		step_num = int(instr[5:8])
		for i in range(step_num):
			step_instr = instr[:5] + '001'
			pos = execute_instr(pos, step_instr, obstacles)
			car.set_pos(pos)
			x_list.append(pos[0])
			y_list.append(pos[1])

			plt.clf()
			collided_cells = update_collided_cells(car.pos, collided_cells, 20, 20)
			plot_collided_cells(collided_cells, 20, 20)
			plt.plot(x_list, y_list)
			plot_collision_box(car.pos)
			car.plot_car()
			plot_obstacles(obstacles)
			setup_arena_plot()
			plt.savefig('pngs/' + str(frame) + '.png')
			frame += 1
	return frame, collided_cells, x_list, y_list


obstacles = [(0, 13, 3, 18), (3, 9, 1, 19), (5, 7, 2, 20), (18, 6, 1, 21), (17, 6, 0, 22), (17, 13, 1, 23), (14, 18, 3, 24)]
init_pos = (2,0,0)
car = Car()
car.set_servo(0)
car.set_pos((init_pos[0]+0.5, init_pos[1]+0.5, 0))
instr_list = generate_instr(init_pos, obstacles)
# instr_list = ['PR|start', 'PR|O0', 'PS|FW143', 'PS|BR090', 'PS|BW034', 'PS|BR090', 'PS|FW003', 'PR|S0', 'PR|S0']

x_list = []
y_list = []
frame = 1
collided_cells = np.zeros((20, 20))
for idx, instr in enumerate(instr_list):
	frame, collided_cells, x_list, y_list = render_instr(car, instr, obstacles, frame, collided_cells, x_list, y_list)

# car.set_angle(45)
# car.plot_car()


# instr_list = ['PS|DM150']



# collided_cells = get_collided_cells(init_pos, instr_list, obstacles, 40, 20)
# plot_collided_cells(collided_cells, 40, 20)
# plot_instr(init_pos, instr_list, obstacles)
# plot_obstacles(obstacles)



# ax = plt.gca()
# ax.set_aspect('equal', adjustable='box')
# ax.set_xlim([-2, 40])
# ax.set_ylim([0, 20])
# ax.set_xticks(np.arange(-2,41,1))
# ax.set_yticks(np.arange(0,21,1))
# ax.grid(which='major', alpha=0.2)
# plt.show()
# exit(0)
