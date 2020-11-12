"""
Agent
This class selects a Navigator class and uses it
to calculate path to goal. When agent discovers obstacles
recalculations are required.
Agent executes until it find no further improvement.
It then collects some statistical data about execution.

Information about the environment (obstacles, cell difficulties)
are found in Env class.

Navigators job is to calculate path to goal using
the corresponding algorithm. When obstacles are found
navigator recalculates the path to goal from current
position.
"""

from timeit import default_timer as timer
from abstract_agent import AbstractAgent
from navigators.ara_navigator import ARANavigator
from navigators.field_d_navigator import FieldDNavigator
from navigators.lrta_navigator import LRTANavigator
from navigators.adstar_navigator import ADNavigator

class Agent(AbstractAgent):
	def __init__(self, env, s_start, s_goal, nav):
		self.navigator = FieldDNavigator(env, s_start, s_goal)
		if nav == "FieldD":
			self.navigator = FieldDNavigator(env, s_start, s_goal)
		elif nav == "LRTA":
			self.navigator = LRTANavigator(env, s_start, s_goal)
		elif nav == "ARA":
			self.navigator = ARANavigator(env, s_start, s_goal)
		elif nav == "AD":
			self.navigator = ADNavigator(env, s_start, s_goal)
		else:
			print("Unknown navigator!")
			exit(0)

	def run(self):
		path_ = []
		visited_ = []
		cost_ = None
		time = 0.0

		return_paths = []
		return_visited = []

		while True:
			start = timer()
			path_generator = self.navigator.run()
			while True:
				try:
					path, visited, cost = next(path_generator)
				except StopIteration:
					break

			end = timer()
			time = time + (end - start) # Time in seconds, e.g. 5.38091952400282
			print(time)

			if cost_ == cost:
				print("no learning..")
				break
			print(path)
			#print(visited)
			print(cost)
			print("still learning..")
			path_ = path
			visited_ = visited
			cost_ = cost
			return_paths.append(path)
			return_visited.append(visited)
			time = 0.0
			wait = input("press enter to continue...")

		#print(return_paths)
		#print(return_visited)
		#wait = input("press enter to continue...")
		return return_paths, return_visited, cost_

	def observe(self):
		return self.Env.get_obstacles(self.position)

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
