import math

import abc
from abc import ABC, abstractmethod

class Motions8Algorithm(metaclass=abc.ABCMeta):
	#def __init__(self):
	#	pass

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

	def cost_no_collision(self, s_start, s_goal):
		#print("   cost:" + str(self.costs[s_goal]))
		#print("   hypot:" + str(math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])))

		return self.costs[s_goal] * math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

	def computeCost(self, s_start, s_goal):
		"""
		Calculate Cost for this motion
		:param s_start: starting node
		:param s_goal: end node
		:return:  Cost for this motion
		:note: Cost function could be more complicate!
		"""

		if self.is_collision(s_start, s_goal):
			return math.inf

		return self.costs[s_goal] * math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])
