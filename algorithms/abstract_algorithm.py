import math

import abc
from abc import ABC, abstractmethod

class AbstractAlgorithm(metaclass=abc.ABCMeta):
	@classmethod
	def __subclasshook__(cls, subclass):
		return (hasattr(subclass, 'run') and
				callable(subclass.run) or
				NotImplemented)

	"""@abc.abstractproperty
	def test(self):
		pass"""

	def __init__(self, s_goal, heuristic_type, y, x, m, obs, border, c):
		self.s_goal = s_goal
		self.heuristic_type = heuristic_type
		self.y = y
		self.x = x # size of the map (y,x)
		self.u_set = m  # feasible input set // usually four or eight
		self.obs = obs
		self.border = border
		self.costs = c

	"""def h(self, s):

		#Calculate heuristic.
		#:param s: current node (state)
		#:return: heuristic function value


		heuristic_type = self.heuristic_type  # heuristic type
		goal = self.s_goal  # goal node
		D = 1
		D2 = round(math.sqrt(2), 5)

		if heuristic_type == "manhattan":
			return D * (abs(goal[0] - s[0]) + abs(goal[1] - s[1]))
		elif heuristic_type == "diagonal":
			#return D * (abs(goal[0] - s[0]) + abs(goal[1] - s[1])) + (D2 - 2 * D) * min(abs(goal[0] - s[0]), abs(goal[1] - s[1]))
			return D * max(abs(goal[0] - s[0]), abs(goal[1] - s[1])) + (D2 - D) * min(abs(goal[0] - s[0]), abs(goal[1] - s[1]))
		#elif heuristic_type == "euclidean":
		#	return D * math.sqrt(abs(goal[0] - s[0]) * abs(goal[0] - s[0]) + abs(goal[1] - s[1]) * abs(goal[1] - s[1]))
		else: # "euclidean"
			return math.hypot(goal[0] - s[0], goal[1] - s[1])"""

	@abstractmethod
	def run(self):
		raise NotImplementedError

	#@abstractmethod
	#def computeCost(self):
	#	raise NotImplementedError

	def h(self, s_start, s_goal):
		heuristic_type = self.heuristic_type  # heuristic type

		#diffCol = abs(s_goal[1] - s_start[1])
		#diffRow = abs(s_goal[0] - s_start[0])

		#return max(math.sqrt((diffCol)*(diffCol) + (diffRow)*(diffRow)), 0.0)

		#return max(math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1]), 0.0)

		if heuristic_type == "manhattan":
			return round(abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1]), 6)
		else:
			return round(math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1]), 6)

	def get_neighbor(self, s):
		"""
		find neighbors of state s that not in obstacles.
		:param s: state
		:return: neighbors
		"""

		nei_list = set()

		for u in self.u_set:
			s_next = tuple([s[i] + u[i] for i in range(2)])
			if s_next not in self.obs:
				#if not self.is_collision(s, s_next):
				nei_list.add(s_next)

		return nei_list
