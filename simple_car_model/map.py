import math
import numpy as np
import time


class Map:
	# 1 in maze is place where we have been, 2 is place where is wall
	def __init__(self, width, height, startPos):
		print("dsdsdsd")
		self.maze = np.zeros(shape=(width, height))
		self.maze[startPos[0], startPos[1]] = 1
		self.currentPosition = startPos
		#print(self.maze)


	def set_forward_value(self, direction):
		# when go up
		if direction[0] == -1:
			self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = 2
		# when go down
		if direction[0] == 1:
			self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = 2
		# when go left
		if direction[1] == -1:
			self.maze[self.currentPosition[0], self.currentPosition[1]-1] = 2
		# when go right
		if direction[1] == 1:
			self.maze[self.currentPosition[0], self.currentPosition[1]+1] = 2


	def set_values(self, direction, leftSensorDistance, rightSensorDistance):
		print("Hello Its me, from the other side")
		# when go up
		if direction[0] == -1:
			#set first values
			print("UP")
			self.currentPosition[0] = self.currentPosition[0] - 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			self.maze[self.currentPosition[0], self.currentPosition[1]-1] = 2
			self.maze[self.currentPosition[0], self.currentPosition[1]+1] = 2
			#set second values
			self.currentPosition[0] = self.currentPosition[0] - 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			if leftSensorDistance < 1  and leftSensorDistance > 0.001:
				self.maze[self.currentPosition[0], self.currentPosition[1]-1] = 2
			if rightSensorDistance < 1  and rightSensorDistance > 0.001:
				self.maze[self.currentPosition[0], self.currentPosition[1]+1] = 2
		#when go down
		if direction[0] == 1:
			#set first values
			print("DOWN")
			self.currentPosition[0] = self.currentPosition[0] + 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			self.maze[self.currentPosition[0], self.currentPosition[1]+1] = 2
			self.maze[self.currentPosition[0], self.currentPosition[1]-1] = 2
			#set second values
			self.currentPosition[0] = self.currentPosition[0] + 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			if leftSensorDistance < 1  and leftSensorDistance > 0.001:
				self.maze[self.currentPosition[0], self.currentPosition[1]+1] = 2
			if rightSensorDistance < 1  and rightSensorDistance > 0.001:
				self.maze[self.currentPosition[0], self.currentPosition[1]-1] = 2
		#when go right
		if direction[1] == 1:
			#set first values
			print("RIGHT")
			self.currentPosition[1] = self.currentPosition[1] + 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = 2
			self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = 2
			#set second values
			self.currentPosition[1] = self.currentPosition[1] + 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			if leftSensorDistance < 1  and leftSensorDistance > 0.001:
				self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = 2
			if rightSensorDistance < 1  and rightSensorDistance > 0.001:
				self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = 2
		#when go left
		if direction[1] == -1:
			#set first values
			print("LEFT")
			self.currentPosition[1] = self.currentPosition[1] - 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = 2
			self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = 2
			#set second values
			self.currentPosition[1] = self.currentPosition[1] - 1
			self.maze[self.currentPosition[0], self.currentPosition[1]] = 1
			if leftSensorDistance < 1  and leftSensorDistance > 0.001:
				self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = 2
			if rightSensorDistance < 1  and rightSensorDistance > 0.001:
				self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = 2

		print(self.maze)

	# return 1 if can go front, 2 if can go left, 3 if can go right
	def check_value(self, direction, frontSensorDistance, leftSensorDistance, rightSensorDistance):
		# when go up
		if direction[0] == -1:
			if frontSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == 0:
					return 1
			if leftSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == 0:
					return 2
			if rightSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == 0:
					return 3
		# when go down
		if direction[0] == 1:
			if frontSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == 0:
					return 1
			if leftSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == 0:
					return 2
			if rightSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == 0:
					return 3
		# when go left
		if direction[1] == -1:
			if frontSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == 0:
					return 1
			if leftSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == 0:
					return 2
			if rightSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == 0:
					return 3
		# when go right
		if direction[1] == 1:
			if frontSensorDistance < 0.001:
				if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == 0:
					return 1
			if leftSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == 0:
					return 2
			if rightSensorDistance < 0.001:
				if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == 0:
					return 3
			

