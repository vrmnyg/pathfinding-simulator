"""
ARA
This class contains navigation logic for ARA algorithm
"""
import math
import utils.env
#import copy
#import queue
#import numpy as np
from timeit import default_timer as timer
from navigators.abstract_navigator import AbstractNavigator

from algorithms.ara import AraStar

class ARANavigator(AbstractNavigator):
	def __init__(self, env, s_start, s_goal):
		self.s_start, self.s_goal = s_start, s_goal
		self.o_start = s_start
		self.Env = env  # class Env
		self.position = self.s_start

		#self.obs = self.Env.getBorder()
		self.border = self.Env.getBorder()
		self.route = []
		self.path = []
		self.visited = []

		self.obs = self.border | self.observe()
		self.print_env()
		self.Env.print_env(self.position, self.s_goal)

		self.totalPathCost = 0.0
		self.costs = {}

		for i in range(0, self.Env.y_range):
			for j in range(0, self.Env.x_range):
				if (i, j) in self.obs:
					self.costs[(i, j)] = math.inf
				else:
					if self.Env.test_obstacle((i, j)):
						self.costs[(i, j)] = math.inf
						sum = 0
						count = 0
						for sn in self.Env.get_neighbor((i, j)):
							nc = self.Env.getCost(sn)
							if nc != math.inf:
								sum = sum + nc
								count = count + 1
						# if count > 0 # ZeroDivisionError
						self.costs[(i, j)] = sum / count
					else:
						self.costs[(i, j)] = self.Env.getCost((i, j))

	def run(self):
		#arastar = AraStar(self.s_goal, 2.5, "euclidean", self.Env.y_range, self.Env.x_range, self.Env.motions, self.obs, self.border)
		self.position = self.o_start
		time = 0.0
		path = 0
		temp_path = []
		temp_visited = []
		first_run = True
		self.totalPathCost = 0
		self.path = []
		self.route = []
		#self.path = []
		while self.s_goal != self.position:
			start = timer()
			arastar = AraStar(self.s_goal, 2.5, "euclidean", self.Env.y_range, self.Env.x_range, self.Env.motions, self.obs, self.border, self.costs)
			path_generator = arastar.run(self.position)
			try:
				while time < 1 or first_run == True:
					path, visited, movementCosts = next(path_generator)
					path.reverse()
					movementCosts.reverse()
					#print(path)
					#print(visited)
					#time = time + 1
					first_run = False
					end = timer()
					time = time + (end - start) # Time in seconds, e.g. 5.38091952400282
					temp_visited = temp_visited + visited[-1]

			except StopIteration:
				pass
			finally:
				del path_generator

			while len(path) > 0:
				step = path.pop(0)
				if step == self.position:
					temp_path.append(step)
					continue

				is_obstacle = self.Env.test_obstacle(step)
				if is_obstacle:
					#self.obs.add(step)
					break
				elif self.is_collision(self.position, step):
					middle_step = (step[0] + 1, step[1])
					is_obs = self.Env.test_obstacle(middle_step)
					if is_obs:
						#self.obs.add(step)
						break
					else:
						movementCosts.pop(0)
						self.position = middle_step
						self.route.append(middle_step)
						self.obs = self.obs | self.observe()

						try:
							for o in self.Env.observe(self.position):
								self.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						self.totalPathCost = self.totalPathCost + self.costs[self.position]
						temp_path.append(middle_step)


						self.position = step
						self.route.append(step)
						self.obs = self.obs | self.observe()

						try:
							for o in self.Env.observe(self.position):
								self.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						self.totalPathCost = self.totalPathCost + self.costs[self.position]
						temp_path.append(step)
				else:
					self.position = step
					#print(self.position)
					movementCost = movementCosts.pop(0)
					self.route.append(self.position)
					self.obs = self.obs | self.observe()
					#arastar.updateObstacles(self.obs)
					self.totalPathCost = self.totalPathCost + movementCost
					temp_path.append(step)

					try:
						for o in self.Env.observe(self.position):
							self.costs[o[0]] = o[1]
					# no need to update costs when keyerror, only full (INT, INT) positions have costs
					except KeyError:
						pass

			self.path.append(temp_path)
			self.visited.append(temp_visited)
			#temp_visited.append(visited)
			temp_path = []
			temp_visited = []
			print(self.position)
			self.print_env()
			print(self.observe())
			#arastar.ChangeEdgeCosts(self.observe())
			time = 0.0
			first_run = True

		yield self.path, self.visited, self.totalPathCost

	def observe(self):
		return self.Env.get_obstacles(self.position)

	def is_collision(self, s_start, s_end):
		"""
		check if the line segment (s_start, s_end) is collision.
		:param s_start: start node
		:param s_end: end node
		:return: True: is collision / False: not collision
		"""

		if s_start in self.obs or s_end in self.obs:
			return True

		if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
			if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
				s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
				if s1 in self.obs:
					return True
		return False

	def print_env(self):
		for i in range(-1, self.Env.y_range+1):
			for j in range(-1, self.Env.x_range+1):
				if (i, j) in self.obs:
					print("#", end='')
				elif (i, j) == self.position:
					print("A", end='')
				elif (i, j) == self.s_goal:
					print("G", end='')
				elif (i, j) == self.o_start:
					print("S", end='')
				elif (i, j) in self.route:
					print("*", end='')
				else:
					print(".", end='')
			print("\n")
