from dataclasses import dataclass
import math

# @dataclass
# class SceneObject:

LINK_X = 0.5	
LINK_Y = 3
LINK_GAP = 9

LINK_LENGTH = math.sqrt(LINK_Y**2 + LINK_X**2)
LINK_ANGLE = math.atan(LINK_X/LINK_Y)

WHEEL_WIDTH = 2.7

PIVOT_GAP = LINK_GAP + 2*LINK_X
PIVOT_OFFSET = LINK_X + 3.8 - WHEEL_WIDTH/2

FRONT_BACK_GAP = 14.5

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
		print(gap_x, gap_y)
		return gap

right_angle = 30
left_angle = math.degrees(__SteeringConverter.rightToLeft(LINK_LENGTH, LINK_ANGLE, LINK_GAP, math.radians(right_angle)))
print(round(left_angle, 3))
print(__SteeringConverter.frontContactGap(math.radians(left_angle), math.radians(right_angle), PIVOT_GAP, PIVOT_OFFSET))
print(__SteeringConverter.frontRightToBackRightGap(math.radians(right_angle), PIVOT_OFFSET, FRONT_BACK_GAP))
print(__SteeringConverter.frontLeftToBackLeftGap(math.radians(left_angle), PIVOT_OFFSET, FRONT_BACK_GAP))