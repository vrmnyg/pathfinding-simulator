"""
AD*
This class contains navigation logic for AD* algorithm
"""

import math
import utils.env
from timeit import default_timer as timer
from navigators.abstract_navigator import AbstractNavigator

from algorithms.adstar import ADStar

class ADNavigator(AbstractNavigator):
	def __init__(self, env, s_start, s_goal):
		self.s_start, self.s_goal = s_start, s_goal
		self.o_start = s_start
		#self.Env = env  # class Env
		self.position = s_start

		#self.obs = self.Env.getBorder()
		#self.border = self.Env.getBorder()
		self.route = []
		self.path = []
		self.visited = []

		#self.obs = self.border | self.observe()
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

		self.dstar = ADStar(s_start, s_goal, 2.5, "euclidean", env, env.y_range, env.x_range, env.motions, obs, border, costs)

	def run(self):
		self.position = self.o_start
		time = 0.0
		path = 0
		first_run = True
		temp_path = []
		self.totalPathCost = 0
		self.path = []
		self.route = []
		self.visited = []

		while self.s_goal != self.position:
			start = timer()
			path_generator = self.dstar.run(self.position)
			try:
				while time < 0.001 or first_run == True:
				#while time < 1 or first_run == True:
					path, visited, movementCosts = next(path_generator)
					#time = time + 1
					first_run = False
					end = timer()
					time = time + (end - start) # Time in seconds, e.g. 5.38091952400282
					print(time)
					print(first_run)
			except StopIteration:
				pass
			finally:
				del path_generator

			while len(path) > 0:
				step = path.pop(0)
				if step == self.position:
					temp_path.append(step)
					continue

				is_obstacle = self.dstar.Env.test_obstacle(step)
				if is_obstacle:
					#self.obs.add(step)
					break
				elif self.is_collision(self.position, step):
					middle_step = (step[0] + 1, step[1])
					is_obs = self.dstar.Env.test_obstacle(middle_step)
					if is_obs:
						#self.obs.add(step)
						break
					else:
						movementCosts.pop(0)
						self.position = middle_step
						self.route.append(middle_step)
						self.dstar.obs = self.dstar.obs | self.dstar.observe(self.position)

						try:
							for o in self.dstar.Env.observe(self.position):
								self.dstar.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						self.totalPathCost = self.totalPathCost + self.dstar.costs[self.position]
						temp_path.append(middle_step)

						self.position = step
						self.route.append(step)
						self.dstar.obs = self.dstar.obs | self.dstar.observe(self.position)

						try:
							for o in self.dstar.Env.observe(self.position):
								self.dstar.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						self.totalPathCost = self.totalPathCost + self.dstar.costs[self.position]
						temp_path.append(step)
						#wait = input("Collision...")
				else:
					self.position = step
					movementCost = movementCosts.pop(0)
					self.route.append(self.position)
					self.dstar.obs = self.dstar.obs | self.dstar.observe(self.position)
					# dstar.updateObstacles(self.obs)

					# keyerror when position has decimals, like (2,45, 2)
					try:
						for o in self.dstar.Env.observe(self.position):
							self.dstar.costs[o[0]] = o[1]
					# no need to update costs when keyerror, only full (INT, INT) positions have costs
					except KeyError:
						pass

					self.totalPathCost = self.totalPathCost + movementCost
					temp_path.append(step)

			#print(self.dstar.g)
			#print(self.dstar.g)
			self.path.append(temp_path)
			#print(self.path)
			#wait = input("prs")
			temp_path = []
			self.visited.append(visited)
			self.dstar.visited = set()
			#print(self.position)
			#self.print_env()
			#print(self.observe())
			self.dstar.ChangeEdgeCosts(self.observe())
			time = 0.0
			first_run = True

		#print(self.dstar.g)
		#self.dstar.init(self.path)
		#self.dstar.g[self.s_start] = math.inf
		#for i in self.dstar.g.keys():
			#self.dstar.g[i] = self.dstar.rhs[i]
		#print(self.dstar.g)
		#self.dstar.rhs[self.dstar.s_goal] = 0.0
		#self.dstar.OPEN[self.dstar.s_goal] = self.dstar.Key(self.dstar.s_goal)
		#self.print_env()
		#print(self.route)
		#wait = input("g")
		yield self.path, self.visited, self.totalPathCost

	def observe(self):
		return self.dstar.Env.get_obstacles(self.position)

	def is_collision(self, s_start, s_end):
		"""
		check if the line segment (s_start, s_end) is collision.
		:param s_start: start node
		:param s_end: end node
		:return: True: is collision / False: not collision
		"""

		if s_start in self.dstar.obs or s_end in self.dstar.obs:
			return True

		if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
			if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
				s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
				if s1 in self.dstar.obs:
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
