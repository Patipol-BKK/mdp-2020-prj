from plotter import *

def pos_to_cell(pos):
	if pos[2] == UP:
		orient = 0
	elif pos[2] == DOWN:
		orient = 1
	elif pos[2] == LEFT:
		orient = 2
	else:
		orient = 3

	return round(pos[0]-10.5), round(pos[1]-10.5), orient


# obstacles = [(4, 10, 3, 4), (12, 3, 3, 6), (3, 10, 0, 5), (5, 3, 3, 3), (3, 17, 3, 2)] 
# instr_list = ['PR|start', 'PR|O3', 'PS|FW030', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW020', 'PS|FL090', 'PS|BW020', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S3', 'PR|O6', 'PS|FW023', 'PS|BR090', 'PS|FW003', 'PS|BL090', 'PS|BW010', 'PS|BL090', 'PS|FW003', 'PS|BR090', 'PS|FW003', 'PR|S6', 'PR|O4', 'PS|BW003', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|FW040', 'PR|S4', 'PR|O2', 'PS|BW013', 'PS|FR090', 'PS|BW003', 'PS|FW030', 'PS|FL090', 'PS|BW020', 'PR|S2', 'PR|O5', 'PS|FL180', 'PS|BW030', 'PS|BL090', 'PR|S5']


init_orient = UP
instr_list = ['PS|BW003','PS|FR050','PS|FW000','PS|BL040','PS|BW005']
cost = 10

init_pos = (10.5, 10.5, 0)

collided_cells = get_collided_cells(init_pos, instr_list)
collided_list = []
for i in range(20):
	for j in range(20):
		if collided_cells[i][j]:
			collided_list.append((i-10, j-10))
# print(collided_list)

plot_collided_cells(collided_cells)
final_pos = plot_instr(init_pos, instr_list)
final_cell_pos = pos_to_cell(final_pos)

collision_dat = [pos_to_cell(init_pos)[2], final_cell_pos, cost, [instr[3:] for instr in instr_list], collided_list]
print(collision_dat)

ax = plt.gca()
ax.set_xticks(np.arange(0,21,1))
ax.set_yticks(np.arange(0,21,1))
ax.grid(which='major', alpha=0.2)
plt.show()