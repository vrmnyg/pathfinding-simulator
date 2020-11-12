"""
ARA* algorithm
https://papers.nips.cc/paper/2003/file/ee8fe9093fbbb687bef15a38facc44d2-Paper.pdf
"""

import math

from algorithms.abstract_algorithm import AbstractAlgorithm
from algorithms.motions8_algorithm import Motions8Algorithm

class AraStar(AbstractAlgorithm, Motions8Algorithm):
	def __init__(self, s_goal, e, heuristic_type, y, x, m, obs, border, c):
		super().__init__(s_goal, heuristic_type, y, x, m, obs, border, c)
		#Motions8Algorithm.__init__(self)

		self.s_start = 0
		self.e = e

		#self.orig_e = e
		self.e_delta = 0.5

		self.g = dict()                                                     # G values in dictionary, key is tuple (y, x) (x, y)???, value is current g
		self.OPEN = dict()                                                  # priority queue / OPEN set, key == node (tuple), value == f-value
		self.PARENT = dict()                                                # relations, key == node, value == node
		self.INCONS = {}                                                    # INCONSISTENT set, nodes that have been visited already

		# basket = {'apple', 'orange', 'apple', 'pear', 'orange', 'banana'}
		self.CLOSED = set()                                                 # CLOSED set

		self.path = []                                                      # planning path
		self.visited = []                                                   # order of visited nodes

	def init(self):
		self.g[self.s_start] = 0.0
		self.g[self.s_goal] = math.inf
		self.OPEN[self.s_start] = self.f_value(self.s_start)
		self.PARENT[self.s_start] = self.s_start

	# f = g + e * h
	def f_value(self, x):
		return self.g[x] + self.e * self.h(x, self.s_goal)

	# return smalles f-value from OPEN-list
	def calc_smallest_f(self):
		s_small = min(self.OPEN, key=self.OPEN.get)
		return s_small, self.OPEN[s_small]

	# get best path, calculate cost
	def extract_path(self):
		path = [self.s_goal]
		s = self.s_goal
		pathCosts = []

		while True:
			cost = self.cost_no_collision(self.PARENT[s], s)
			pathCosts.append(cost)

			s = self.PARENT[s]
			path.append(s)

			if s == self.s_start:
				break

		return list(path), pathCosts

	# update e from paper
	# this affects the fact if execution continues or not
	def update_e(self):
		v = math.inf
		if self.OPEN:
			v = min(self.g[s] + self.h(s, self.s_goal) for s in self.OPEN)
		if self.INCONS:
			v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))
		return min(self.e, self.g[self.s_goal] / v)

	# improvepath from paper
	def ImprovePath(self):
		visited_each = []
		while True:
			s, f_small = self.calc_smallest_f()

			if self.f_value(self.s_goal) <= f_small:
				break

			self.OPEN.pop(s)
			self.CLOSED.add(s)

			for s_n in self.get_neighbor(s):
				if s_n in self.obs:
					continue

				new_cost = self.g[s] + self.computeCost(s, s_n)

				if s_n not in self.g or new_cost < self.g[s_n]:
					self.g[s_n] = new_cost
					self.PARENT[s_n] = s
					visited_each.append(s_n)

					if s_n not in self.CLOSED:
						self.OPEN[s_n] = self.f_value(s_n)
					else:
						self.INCONS[s_n] = 0.0

		self.visited.append(visited_each)

	def run(self, s_start):
		self.s_start = s_start
		self.init()

		while self.update_e() > 1:                                          # continue condition
			self.ImprovePath()
			path, costs = self.extract_path()
			self.e -= self.e_delta                                       	# increase weight
			self.OPEN.update(self.INCONS)
			self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

			self.INCONS = dict()
			self.CLOSED = set()

			yield path, self.visited, costs
