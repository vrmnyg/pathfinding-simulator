"""
LSS-LRTA algorithm
http://idm-lab.org/bib/abstracts/papers/jaamas09.pdf
"""

import math
import utils.queue as queue
import copy

from algorithms.abstract_algorithm import AbstractAlgorithm
from algorithms.motions8_algorithm import Motions8Algorithm

class LrtAStarN(AbstractAlgorithm, Motions8Algorithm):
	def __init__(self, s_goal, N, heuristic_type, env, y, x, m, obs, border, c):
		AbstractAlgorithm.__init__(self, s_goal, heuristic_type, y, x, m, obs, border, c)

		self.s_start = 0
		self.Env = env  # class Env
		self.N = N  # number of expand nodes each iteration
		self.final_c = [0]

		self.visited = []  # order of visited nodes in planning
		#self.path = []  # path of each iteration
		self.h_table = {}  # h_value table

		#self.Env.print_env(s_start, s_goal)
		self.init()
		#print(self.h_table)
		#wait = input("h_table")

	def init(self):
		for i in range(self.y):
			for j in range(self.x):
				self.h_table[(i, j)] = self.h((i, j), self.s_goal)

	def observe(self, s):
		return self.Env.get_obstacles(s)

	# get path based on parent - child relation, calculate path cost
	def extract_path(self, x_start, parent):
		path_back = [self.s_goal]
		x_current = self.s_goal
		pathCosts = []

		while True:
			cost = self.cost_no_collision(parent[x_current], x_current)
			pathCosts.append(cost)

			x_current = parent[x_current]
			path_back.append(x_current)

			if x_current == x_start:
				break

		return list(reversed(path_back)), pathCosts

	# astar part of the algorithm, find out boundaries of the search
	def AStar(self, x_start, N):
		OPEN = queue.QueuePrior()  # OPEN set
		OPEN.put(x_start, self.h(x_start, self.s_goal))
		CLOSED = []  # CLOSED set
		g_table = {x_start: 0, self.s_goal: math.inf}  # Cost to come
		PARENT = {x_start: x_start}  # relations
		count = 0  # counter

		while not OPEN.empty():
			count += 1
			s = OPEN.get()
			CLOSED.append(s)

			if s == self.s_goal:  # reach the goal node
				self.visited.extend(copy.deepcopy(CLOSED))
				c, costs = self.extract_path(x_start, PARENT)
				self.final_c = costs
				return "FOUND", c

			for s_n in self.get_neighbor(s):
				#print(" neighboring node " + str(s_n))
				if s_n not in CLOSED:
					new_cost = g_table[s] + self.computeCost(s, s_n)
					#print("	current node g: " + str(g_table[s]) + " from current to n cost: " + str(self.computeCost(s, s_n)) + " sum: " + str(new_cost))
					if s_n not in g_table:
						#print("	" + str(s_n) + " not in g-table")
						g_table[s_n] = math.inf
					if new_cost < g_table[s_n]:  # better route found, update
						#print("	" + str(new_cost) + " < " + str(g_table[s_n]) + " , update")
						g_table[s_n] = new_cost
						PARENT[s_n] = s
						#print("	add to open: " + str(s_n) + " f: " + str(g_table[s_n] + self.h_table[s_n]))
						#OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])
						if OPEN.exists(s_n):
							OPEN.update(s_n, g_table[s_n] + self.h_table[s_n])
						else:
							OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])

			if count == N:  # expand needed CLOSED nodes
				break

		self.visited.extend(copy.deepcopy(CLOSED))  # visited nodes in each iteration
		return OPEN, CLOSED

	# update heuristics with dijkstra algorithm
	def dijkstra(self, OPEN, CLOSED):
		h_value = {}

		for s in CLOSED:
			 # initialize h_value of CLOSED nodes (local search space), these are the values that get updated
			 #print(s)
			 self.h_table[s] = math.inf

		# OPEN nodes and h-values : key is (y,x) : value is h-value
		open_unsorted = {s[1]:self.h_table[s[1]] for s in OPEN.enumerate()}

		while CLOSED:
		#while open_unsorted:
			# get the best key from OPEN (lowest h-value)
			key = min(open_unsorted, key=open_unsorted.get)
			del open_unsorted[key]

			# remove key from closed if it is there
			try:
				CLOSED.remove(key)
			except ValueError:
				pass

			# update heuristics
			# is looping through all open list necessary?
			for k_n in self.get_neighbor(key):
				#print(" neighboring node " + str(k_n))
				if k_n in CLOSED and self.h_table[k_n] > self.computeCost(key, k_n) + self.h_table[key]:
					self.h_table[k_n] = self.computeCost(key, k_n) + self.h_table[key]
					#if k_n not in open_unsorted:
					open_unsorted[k_n] = self.h_table[k_n]
					h_value[k_n] = self.h_table[k_n]

		return h_value

	# extract path
	def extract_path_in_CLOSE(self, s_start, h_value):
		path = [s_start]
		s = s_start
		pathCosts = []

		while True:
			h_list = {}

			for s_n in self.get_neighbor(s):
				if s_n in h_value:
					h_list[s_n] = h_value[s_n]
				else:
					h_list[s_n] = self.h_table[s_n]

			s_key = min(h_list, key=h_list.get)  # move to the smallest node with min h_value

			cost = self.cost_no_collision(s, s_key)
			pathCosts.append(cost)

			path.append(s_key)  # generate path
			s = s_key  # use end of this iteration as the start of next

			if s_key not in h_value:
				return s_key, path, pathCosts

	def run(self, s_start):
		self.s_start = s_start  # initialize start node

		while True:
			#print(self.h_table)
			# self.N is max number of expanded nodes, affects the search greatly!
			OPEN, CLOSED = self.AStar(s_start, self.N)  # OPEN, CLOSED sets in each iteration

			if OPEN == "FOUND":  # reach the goal node
				yield CLOSED, self.visited, self.final_c
				break

			# update h-values based on OPEN and CLOSED lists from AStar
			h_value = self.dijkstra(OPEN, CLOSED)

			# find the path
			self.s_start, path_k, costs = self.extract_path_in_CLOSE(s_start, h_value)  # x_init -> expected node in OPEN set

			yield path_k, self.visited, costs
