"""
LRTA
This class contains navigation logic for LRTA algorithm
"""
import math
import utils.env
#import numpy as np
#from timeit import default_timer as timer
from navigators.abstract_navigator import AbstractNavigator

from algorithms.lrta import LrtAStarN

class LRTANavigator(AbstractNavigator):
	def __init__(self, env, s_start, s_goal):
		#self.s_start, self.s_goal = s_start, s_goal
		self.o_start = s_start
		#self.Env = env  # class Env
		self.position = s_start

		self.route = []
		self.path = []
		self.visited = []

		#self.obs = self.Env.getBorder()
		#self.print_env()
		#self.Env.print_env(self.position, self.s_goal)

		self.totalPathCost = 0

		border = env.getBorder()
		obs = border | env.get_obstacles(s_start)

		costs = {}

		for i in range(0, env.y_range):
			for j in range(0, env.x_range):
				if (i, j) in obs:
					costs[(i, j)] = math.inf
				else:
					if env.test_obstacle((i, j)):
						costs[(i, j)] = math.inf
						sum = 0
						count = 0
						for sn in env.get_neighbor((i, j)):
							nc = env.getCost(sn)
							if nc != math.inf:
								sum = sum + nc
								count = count + 1
						# if count > 0 # ZeroDivisionError
						costs[(i, j)] = sum / count
					else:
						costs[(i, j)] = env.getCost((i, j))

		self.lrta = LrtAStarN(s_goal, 5, "euclidean", env, env.y_range, env.x_range, env.motions, obs, border, costs)
		#lrta = LrtAStarN(self.s_goal, 4, "diagonal", self.Env.y_range, self.Env.x_range, self.Env.motions, self.obs, self.border, self.costs)
		#lrta = LrtAStarN(self.s_goal, 4, "manhattan", self.Env.y_range, self.Env.x_range, self.Env.motions, self.obs, self.border, self.costs)

	def run(self):
		self.position = self.o_start
		path = 0
		temp_path = []
		self.totalPathCost = 0
		self.path = []
		self.route = []
		#self.lrta.visited = []
		while self.lrta.s_goal != self.position:
			path_generator = self.lrta.run(self.position)
			try:
				#path = next(path_generator)
				path, visited, movementCosts = next(path_generator)
			except StopIteration:
				break
			finally:
				del path_generator

			#temp_path.append(self.position)
			while len(path) > 0:
				step = path.pop(0)
				if step == self.position:
					temp_path.append(step)
					continue

				is_obstacle = self.lrta.Env.test_obstacle(step)
				if is_obstacle:
					#self.path.append(temp_path)
					#self.visited.append(visited)
					#self.lrta.visited = []
					break
				elif self.is_collision(self.position, step):
					#break
					middle_step = (step[0] + 1, step[1])
					is_obs = self.lrta.Env.test_obstacle(middle_step)
					if is_obs:
						#self.obs.add(step)
						break
					else:
						movementCosts.pop(0)
						self.position = middle_step
						self.route.append(middle_step)
						self.lrta.obs = self.lrta.obs | self.lrta.observe(self.position)

						try:
							for o in self.lrta.Env.observe(self.position):
								self.lrta.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						#self.lrta.updateObstacles(self.lrta.obs)
						self.totalPathCost = self.totalPathCost + self.lrta.costs[self.position]
						temp_path.append(middle_step)

						self.position = step
						self.route.append(step)
						self.lrta.obs = self.lrta.obs | self.lrta.observe(self.position)

						try:
							for o in self.lrta.Env.observe(self.position):
								self.lrta.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						#self.lrta.updateObstacles(self.lrta.obs)
						self.totalPathCost = self.totalPathCost + self.lrta.costs[self.position]
						temp_path.append(step)
						#wait = input("Collision...")
				else:
					self.position = step
					movementCost = movementCosts.pop(0)
					self.route.append(self.position)
					self.lrta.obs = self.lrta.obs | self.lrta.observe(self.position)

					# keyerror when position has decimals, like (2,45, 2)
					try:
						for o in self.lrta.Env.observe(self.position):
							self.lrta.costs[o[0]] = o[1]
					# no need to update costs when keyerror, only full (INT, INT) positions have costs
					except KeyError:
						pass

					#self.lrta.updateObstacles(self.lrta.obs)
					self.totalPathCost = self.totalPathCost + movementCost
					temp_path.append(step)

					#print(self.lrta.costs)

					#wait = input("press enter to continue...")

			self.path.append(temp_path)
			temp_path = []
			self.visited.append(visited)
			self.lrta.visited = []
			#yield self.path, self.visited, self.totalPathCost
		#wait = input("PRESS ENTER TO CONTINUE.")
		yield self.path, self.visited, self.totalPathCost
		self.visited = []
		#self.visited.append(self.visited)

	def is_collision(self, s_start, s_end):
		"""
		check if the line segment (s_start, s_end) is collision.
		:param s_start: start node
		:param s_end: end node
		:return: True: is collision / False: not collision
		"""

		if s_start in self.lrta.obs or s_end in self.lrta.obs:
			return True

		if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
			if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
				s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
				if s1 in self.lrta.obs:
					return True
		return False

	def print_env(self):
		for i in range(-1, self.lrta.Env.y_range+1):
			for j in range(-1, self.lrta.Env.x_range+1):
				if (i, j) in self.lrta.obs:
					print("#", end='')
				elif (i, j) == self.position:
					print("A", end='')
				elif (i, j) == self.lrta.s_goal:
					print("G", end='')
				elif (i, j) == self.o_start:
					print("S", end='')
				elif (i, j) in self.route:
					print("*", end='')
				else:
					print(".", end='')
			print("\n")
