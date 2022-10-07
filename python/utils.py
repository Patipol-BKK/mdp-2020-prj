import numpy as np
import matplotlib.pyplot as plt

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class Polygon:
	def __init__(self, pos_list, anchor_pos):
		self.anchor_pos = anchor_pos
		self.polar_pos_list = []
		for pos in pos_list:
			polar_pos = cart2pol(pos[0] - anchor_pos[0], pos[1] - anchor_pos[1])
			self.polar_pos_list.append(polar_pos)
		self.heading = 0

	def set_angle(self, angle):
		self.heading = angle

	def set_pos(self, pos):
		self.anchor_pos = pos
		if len(pos) == 3:
			self.set_angle(pos[2])

	def get_coordinates(self):
		cart_pos_list = [pol2cart(polar_pos[0], polar_pos[1] - np.radians(self.heading)) for polar_pos in self.polar_pos_list]
		return [(cart_pos[0] + self.anchor_pos[0], cart_pos[1] + self.anchor_pos[1]) for cart_pos in cart_pos_list]

	def plot(self, color, fill, alpha):
		cart_pos = [pol2cart(polar_pos[0], polar_pos[1] - np.radians(self.heading)) for polar_pos in self.polar_pos_list]
		x_list = [pos[0] + self.anchor_pos[0] for pos in cart_pos]
		y_list = [pos[1] + self.anchor_pos[1] for pos in cart_pos]
		
		x_list.append(cart_pos[0][0] + self.anchor_pos[0])
		y_list.append(cart_pos[0][1] + self.anchor_pos[1])

		plt.plot(x_list, y_list, color=color)
		plt.fill(x_list, y_list, color=fill, alpha=alpha)

class Car:
	def __init__(self):
		self.pos = (0, 0, 0)
		self.body = Polygon([(-0.55, -0.3), (0.55, -0.3), (0.55, 2), (-0.55, 2)], (0, 0))

		self.servo_angle = 0
		self.front_wheel_anchors = Polygon([(-0.5, 1.45), (0.5, 1.45)], (0, 0))
		self.front_left_wheel = Polygon([(-0.66, 1.75), (-0.93, 1.75), (-0.93, 1.15), (-0.66, 1.15)], (-0.5, 1.45))
		self.front_right_wheel = Polygon([(0.66, 1.75), (0.93, 1.75), (0.93, 1.15), (0.66, 1.15)], (0.5, 1.45))

		self.back_left_wheel = Polygon([(-0.66, 0.3), (-0.93, 0.3), (-0.93, -0.3), (-0.66, -0.3)], (0, 0))
		self.back_right_wheel = Polygon([(0.66, 0.3), (0.93, 0.3), (0.93, -0.3), (0.66, -0.3)], (0, 0))

	def set_pos(self, pos):
		self.pos = pos
		self.body.set_pos(pos)

		self.front_wheel_anchors.set_pos(pos)
		self.front_left_wheel.set_pos(self.front_wheel_anchors.get_coordinates()[0])
		self.front_left_wheel.set_angle(self.servo_angle + pos[2])
		self.front_right_wheel.set_pos(self.front_wheel_anchors.get_coordinates()[1])
		self.front_right_wheel.set_angle(self.servo_angle + pos[2])

		self.back_left_wheel.set_pos(pos)
		self.back_right_wheel.set_pos(pos)

	def set_angle(self, angle):
		self.pos = (self.pos[0], self.pos[1], angle)
		self.set_pos(self.pos)

	def set_servo(self, servo_angle):
		self.servo_angle = servo_angle

		self.front_left_wheel.set_pos(self.front_wheel_anchors.get_coordinates()[0])
		self.front_left_wheel.set_angle(self.servo_angle + self.pos[2])
		self.front_right_wheel.set_pos(self.front_wheel_anchors.get_coordinates()[1])
		self.front_right_wheel.set_angle(self.servo_angle + self.pos[2])

	def plot_car(self):
		self.body.plot('gray', 'silver', 1)
		self.front_left_wheel.plot('red', 'salmon', 1)
		self.front_right_wheel.plot('red', 'salmon', 1)
		self.back_left_wheel.plot('red', 'salmon', 1)
		self.back_right_wheel.plot('red', 'salmon', 1)
		print(self.body.get_coordinates())

def setup_arena_plot():
	ax = plt.gca()
	ax.set_xticks(np.arange(0,21,1))
	ax.set_yticks(np.arange(0,21,1))
	ax.grid(which='major', alpha=0.2)
	# ax = plt.gca()
	ax.set_aspect('equal', adjustable='box')
	ax.set_xlim([0, 20])
	ax.set_ylim([0, 20])