"""
AD* algorithm
https://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf
"""

import math

from algorithms.abstract_algorithm import AbstractAlgorithm
from algorithms.motions8_algorithm import Motions8Algorithm

class ADStar(AbstractAlgorithm, Motions8Algorithm):
	def __init__(self, s_start, s_goal, eps, heuristic_type, env, y, x, m, obs, border, c):
		AbstractAlgorithm.__init__(self, s_goal, heuristic_type, y, x, m, obs, border, c)

		self.s_start = s_start
		#print("init")
		self.Env = env  # class Env
		self.eps = eps
		self.eps_delta = 0.5
		self.orig_eps = eps

		# dictionaries
		self.g, self.rhs, self.OPEN = {}, {}, {}

		# set all rhs and g values to infinity
		for i in range(0, self.y):
			for j in range(0, self.x):
				self.rhs[(i, j)] = math.inf
				self.g[(i, j)] = math.inf

		# start searching from goal by setting goal rhs to zero
		self.rhs[self.s_goal] = 0.0

		self.OPEN[self.s_goal] = self.Key(self.s_goal)
		self.CLOSED, self.INCONS = set(), dict()

		# visited nodes (node gets added when OPEN is popped)
		self.visited = set()

	# update g-table
	def fix(self, s):
		if self.g[s] > self.rhs[s]:
			self.g[s] = self.rhs[s]

			for sn in self.get_neighbor(s):
				self.UpdateState(sn)
		else:
			self.g[s] = math.inf

			for sn in self.get_neighbor(s):
				self.UpdateState(sn)
			self.UpdateState(s)

	def init(self):
		# dictionaries
		self.g, self.rhs, self.OPEN = {}, {}, {}

		# set all rhs and g values to infinity
		for i in range(0, self.y):
			for j in range(0, self.x):
				self.rhs[(i, j)] = math.inf
				self.g[(i, j)] = math.inf

		# start searching from goal by setting goal rhs to zero
		self.rhs[self.s_goal] = 0.0

		self.OPEN[self.s_goal] = self.Key(self.s_goal)
		self.CLOSED, self.INCONS = set(), dict()

		# visited nodes (node gets added when OPEN is popped)
		self.visited = set()

		# check route, fix path if needed
		"""for r in route:
			for s in r:
				#if self.rhs[s] != self.g[s]:
				if self.g[s] > self.rhs[s]:
					self.g[s] = self.rhs[s]

					for sn in self.get_neighbor(s):
						self.UpdateState(sn)
				else:
					self.g[s] = math.inf

					for sn in self.get_neighbor(s):
						self.UpdateState(sn)
					self.UpdateState(s)"""

	#return list
	def Key(self, s):
		if self.g[s] > self.rhs[s]:
			return [self.rhs[s] + self.eps * self.h(self.s_start, s), self.rhs[s]]
			#return [self.rhs[s] + self.h(self.s_start, s), self.rhs[s]]
		else:
			return [self.g[s] + self.h(self.s_start, s), self.g[s]]

	def observe(self, s):
		return self.Env.get_obstacles(s)

	# keys are used to rank cells in OPEN list
	def TopKey(self):
		s = min(self.OPEN, key=self.OPEN.get)
		return s, self.OPEN[s]

	def testLooping(self):
		path = [self.s_start]
		s = self.s_start
		pathCosts = []

		while True:
			g_list = {}
			for x in self.get_neighbor(s):
				if not self.is_collision(s, x):
					g_list[x] = self.g[x]
			s_parent = s
			s = min(g_list, key=g_list.get)

			cost = self.cost_no_collision(s_parent, s)
			pathCosts.append(cost)

			path.append(s)

			# test looping
			if len(path) > 3 and path[-1] == path[-3]:
				looping_s = path[-1]
				#print("looping_s: ")
				#print(looping_s)
				#print(path)
				del path[-1]
				#del path[-1]
				#del path[-1]
				#print(path)
				return True, looping_s
			if s == self.s_goal:
				break
		return False, None

	# find the best path, calculate cost of path
	def extract_path(self):
		path = [self.s_start]
		s = self.s_start
		pathCosts = []

		while True:
			g_list = {}
			for x in self.get_neighbor(s):
				if not self.is_collision(s, x):
					g_list[x] = self.g[x]
			s_parent = s
			s = min(g_list, key=g_list.get)

			cost = self.cost_no_collision(s_parent, s)
			pathCosts.append(cost)

			path.append(s)

			#print(s)

			if s == self.s_goal:
				break

		return list(path), pathCosts

	def UpdateState(self, s):
		# for all nodes except goal node
		if s != self.s_goal:
			# set rhs to infinity // find the best route to s from neighbors
			self.rhs[s] = math.inf

			for x in self.get_neighbor(s):
				self.rhs[s] = min(self.rhs[s], self.g[x] + self.computeCost(x, s))

		if s in self.OPEN:
			self.OPEN.pop(s)

		if self.g[s] != self.rhs[s]:
			if s not in self.CLOSED:
				self.OPEN[s] = self.Key(s)
			else:
				self.INCONS[s] = 0

	def ComputeOrImprovePath(self):
		#if self.rhs[self.s_start] == self.g[self.s_start]:
			#self.g[self.s_start] = math.inf

		while True:
			s, v = self.TopKey()
			if v >= self.Key(self.s_start) and self.rhs[self.s_start] == self.g[self.s_start]:
				break

			self.OPEN.pop(s)
			self.visited.add(s)

			print(s)

			if self.g[s] > self.rhs[s]:
				self.g[s] = self.rhs[s]
				self.CLOSED.add(s)

				for sn in self.get_neighbor(s):
					self.UpdateState(sn)
			else:
				self.g[s] = math.inf

				for sn in self.get_neighbor(s):
					self.UpdateState(sn)
				self.UpdateState(s)

	# update g-table when obstacles are found
	def ChangeEdgeCosts(self, s):
		self.eps = self.orig_eps
		for i in s:
			if i not in self.obs:
				self.obs.add(i)
				self.g[i] = math.inf
				self.rhs[i] = math.inf
			#else:
			#	self.obs.remove(i)
			#	self.UpdateState(i)
			if i not in self.border:
				for sn in self.get_neighbor(i):
					self.UpdateState(sn)

	def run(self, s_start):
		self.s_start = s_start
		while True:
			if self.eps < 1.0:
				break

			print("running with e: " + str(self.eps))
			#print(self.OPEN)

			self.ComputeOrImprovePath()
			#self.visited = set()

			loop, sl = self.testLooping()
			if loop:
				self.init()
				self.ComputeOrImprovePath()

			# not functioning yet
			"""loop, sl = self.testLooping()
			while loop:
				self.fix(sl)
				self.ComputeOrImprovePath()
				loop, sl = self.testLooping()
				print(sl)
				wait = input("ss")"""

			path, pathCosts = self.extract_path()

			# if changes in edge costs are detected
				# for all directed edges (u, v) with changed edge costs
				# Update the edge cost c(u, v);
				# UpdateState(u);
			# if significant edge cost changes were observed
				# increase e or replan from scratch;
			#self.ChangeEdgeCosts()

			self.eps -= self.eps_delta
			self.OPEN.update(self.INCONS)
			for s in self.OPEN:
				self.OPEN[s] = self.Key(s)
			self.CLOSED = set()

			yield path, list(self.visited), pathCosts
			#wait = input("PRESS ENTER TO CONTINUE.")
