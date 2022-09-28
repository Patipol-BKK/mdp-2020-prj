from plotter import *

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

instr_list = ['PS|FW''PS|DM120', 'PS|FL060', 'PS|FR060', 'PS|DM225', 'PS|FR023', 'PS|FR067', 'PS|FW045', 'PS|FL090', \
			  'PS|FW025', 'PS|FL090', 'PS|FW090', 'PS|FL090', 'PS|DR200', 'PS|FW070', 'PS|FL090', 'PS|FW020', 'PS|FR090', 'PS|DR100']
init_pos = (0.5, 10.5, 90)


obstacles = [(3, 15, 3), (12, 6, 3), (19, 6, 2)]
instr_list = ['PR|start', 'PR|O0', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW080', 'PS|FL090', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PR|S0', 'PR|O0', 'PS|FW003', 'PS|BR090', 'PS|FW046', 'PS|BR090', 'PS|FW003', 'PR|S0', 'PR|O0', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW050', 'PS|FL090', 'PS|FW050', 'PR|S0']
init_pos = (2.5, 0.5, 0)

# obstacles = [(4, 10, 3, 4), (12, 3, 3, 6), (3, 10, 0, 5), (5, 3, 3, 3), (3, 17, 3, 2)] 
# instr_list = ['PR|start', 'PR|O3', 'PS|FW030', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW020', 'PS|FL090', 'PS|BW020', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S3', 'PR|O6', 'PS|FW023', 'PS|BR090', 'PS|FW003', 'PS|BL090', 'PS|BW010', 'PS|BL090', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S6', 'PR|O4', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|FW040', 'PR|S4', 'PR|O2', 'PS|BW013', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|BW020', 'PR|S2', 'PR|O5', 'PS|FL180', 'PS|BW030', 'PS|BL090', 'PR|S5']

collided_cells = get_collided_cells((2.5, 0.5, 0), instr_list, obstacles, 20, 20)

plot_collided_cells(collided_cells, 20, 20)
plot_instr((2.5, 0.5, 0), instr_list, obstacles)
plot_obstacle(obstacles)
ax = plt.gca()
ax.set_xticks(np.arange(0,21,1))
ax.set_yticks(np.arange(0,21,1))
ax.grid(which='major', alpha=0.2)
plt.show()


# instr_list = ['PS|DM150']



# collided_cells = get_collided_cells(init_pos, instr_list, obstacles, 40, 20)
# plot_collided_cells(collided_cells, 40, 20)
# plot_instr(init_pos, instr_list, obstacles)
# plot_obstacle(obstacles)



# ax = plt.gca()
# ax.set_aspect('equal', adjustable='box')
# ax.set_xlim([-2, 40])
# ax.set_ylim([0, 20])
# ax.set_xticks(np.arange(-2,41,1))
# ax.set_yticks(np.arange(0,21,1))
# ax.grid(which='major', alpha=0.2)
# plt.show()
# exit(0)
