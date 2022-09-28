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


# obstacles = [(4, 10, 3, 4), (12, 3, 3, 6), (3, 10, 0, 5), (5, 3, 3, 3), (3, 17, 3, 2)] 
# instr_list = ['PR|start', 'PR|O3', 'PS|FW030', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW020', 'PS|FL090', 'PS|BW020', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S3', 'PR|O6', 'PS|FW023', 'PS|BR090', 'PS|FW003', 'PS|BL090', 'PS|BW010', 'PS|BL090', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S6', 'PR|O4', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|FW040', 'PR|S4', 'PR|O2', 'PS|BW013', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|BW020', 'PR|S2', 'PR|O5', 'PS|FL180', 'PS|BW030', 'PS|BL090', 'PR|S5']

def generate_collision_data(orient, instr_list, cost):
	init_pos = (10.5, 10.5, 0)
	# init_pos = (0.93, 2, 0)

	collided_cells = get_collided_cells(init_pos, instr_list)
	collided_list = []
	for i in range(20):
		for j in range(20):
			if collided_cells[i][j]:
				collided_list.append((i-10, j-10))
				# collided_list.append((i, j))
	# print(collided_list)

	plot_collided_cells(collided_cells)
	final_pos = plot_instr(init_pos, instr_list)
	final_cell_pos = pos_to_cell(final_pos)

	collision_dat = [pos_to_cell(init_pos)[2], final_cell_pos, cost, [instr[3:] for instr in instr_list], collided_list]
	# print(collision_dat)
	return collision_dat

instr_list = ['PS|FL062','PS|BR056','PS|FL062']
print(generate_collision_data(UP, instr_list, 0))
plt.show()
exit(0)
moves_list = [(['PS|FW010'], 1), \
			  (['PS|BW010'], 1), \
			  (['PS|FL090'], 5), \
			  (['PS|BL090'], 5), \
			  (['PS|BW003','PS|FR090','PS|BW003'], 8), \
			  (['PS|FW003','PS|BR090','PS|FW003'], 8), \
			  # (['PS|BW003','PS|FR050','PS|BL040','PS|BW005'], 12), \
			  # (['PS|FW003','PS|BR050','PS|FL040','PS|FW005'], 12), \
			  # (['PS|FL062','PS|BR056','PS|FL062'], 10), \
			  # (['PS|FR058','PS|BL064','PS|FR058'], 10)
			 ]

collision_dat_list = []
with open("possible_moves.txt",'w',encoding = 'utf-8') as f:
	for moves in moves_list:
		instr_list = moves[0]
		cost = moves[1]
		for i in range(4):
			collision_dat_list.append(generate_collision_data(i, instr_list, cost))
	f.write(str(collision_dat_list))
	f.close()
