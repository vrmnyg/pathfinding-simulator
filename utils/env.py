"""
Env 2D
This class contains all information about the environment
the Agent navigates.
All cells have difficulty rating that defines how difficult
the cell is to enter. 1 is default difficulty.
8 is eight times more difficult.
This class contains different ways for creating maps
TODO: way to automatically create obstacles.
"""

import math
import random
# remove this if want to use random environments
random.seed(10)

class Env:
	def __init__(self, y_range, x_range, motions8=False):
		#self.x_range = 6  # size of background
		#self.y_range = 4
		self.x_range = x_range  # size of background
		self.y_range = y_range
		self.motions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
		if motions8 == True:
			self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
		self.border = self.border_map()
		self.obs_map = self.obs_map()
		self.obs = self.border | self.obs_map

		# when using constant values
		self.constant = 2

		self.costs = {}

		# use list to create anvironment, one item is one cell difficulty value
		# make sure map size (in main) is the same or smaller when using lists to create environment
		self.costArray = [
			1,8.03,1,1,1,4.38,1,1,1,1,9.53,1,1,5.09,1,7.89,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1.07,1,1,1,1,1,1,1,1,1,1,1,1,6.32,1,1,1,1,
			1,1.52,1,1,1,1,1,1,9.74,3.45,1,2.42166,2.135,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,9.93,6.43,1,1,5.29,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1.544,3.72,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1.05,1,1,1.83,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,4.2,1,1,1,2.91,1,1.38857,1,1,1,1.76,1,1,1,1,1,1,1,1,1,1,1,1,1,4.19,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4.43,1,1,1,1,
			1,7.94,1.91,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,9.84,1,1,1,1,1,1,6.02,6.37,1,7.5,1,1,1,1,1,1,1,1,1,1,4.6,1,7.91,
			1,1,1,1,1,7.43,1,1,1,1,1,1,1,1.82,7.0,1,1,1,1,1,1,1,1,1,9.79,1,1,7.51,1,1,1,3.38,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			4.07,1,1,1,1,1,1,1.85,1,1,1,3.9,1,1,1,1,1,7.48,1,1,6.66,1,1,1,1,1,1,1,3.84,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,3.59,1,1,1,1,1,1,1,1,1,1,1,1,4.09,1,1,2.23,1,1.28,1,1,1,6.87,1,1,1,4.37,1,1,1,1,1,1,1,1,1,1,1,3.05,3.02,1.9,1,1,1,1.24,1,1,1
		]

		# make sure size is the same
		"""self.costArray = [
		3, 1, 1, 1, 3, 1, 4,
		1, 1, 1, 2, 1, 3, 1,
		1, 2, 2, 1, 4, 1, 2,
		1, 2, 1, 4, 1, 6, 1,
		2, 1, 3, 1, 2, 1, 9
		]"""
		self.newMapfromArray()

		#self.newMapOfConstant()
		#self.newMapOfRandom(10, 0.15)

		#self.costs[(0, 44)] = 9
		#self.costs[(2, 13)] = 1
		#self.newMapOfBinary(10, 0.15)
		#self.newMapFractal(10, 0.2, 0.6, 1)
		#self.costs[(2, 2)] = 100
		#self.costs[(2, 3)] = 50
		#print("costs: ")
		#print(self.costs)

	#map[y][x] = costArray[y * WIDTH + x];

	def newMapOfConstant(self):
		for i in range(0, self.y_range):
			for j in range(0, self.x_range):
				if (i, j) in self.obs:
					self.costs[(i, j)] = math.inf
				else:
					self.costs[(i, j)] = self.constant

	def newMapfromArray(self):
		for i in range(0, self.y_range):
			for j in range(0, self.x_range):
				if (i, j) in self.obs:
					self.costs[(i, j)] = math.inf
				else:
					self.costs[(i, j)] = self.costArray[i * self.x_range + j]

	def newMapOfBinary(self, maxCost, density):
		for i in range(0, self.y_range):
			for j in range(0, self.x_range):
				if (i, j) in self.obs:
					self.costs[(i, j)] = math.inf
				else:
					if random.randint(0, 9999) < 10000 * density:
						#self.costs[(i, j)] = random.randint(1, maxCost)
						self.costs[(i, j)] = maxCost
					else:
						self.costs[(i, j)] = 1

	#random.randint(3, 9)
	def newMapOfRandom(self, maxCost, density):
		for i in range(0, self.y_range):
			for j in range(0, self.x_range):
				if (i, j) in self.obs:
					self.costs[(i, j)] = math.inf
				else:
					if random.randint(0, 9999) < 10000 * density:
						#self.costs[(i, j)] = random.randint(1, maxCost)
						self.costs[(i, j)] = round(1 + random.randint(0, (1000*(maxCost-1)+1))/1000, 2)
					else:
						self.costs[(i, j)] = 1

	# TODO
	# creates an N-Tiling where the costs are determined by fractal map generation,
	# values are between 1 and maxCost,
	# 0 <= bump_factor <= 1
	# 0 <= uniformity <= 1, where closer to 1 means that the maps overall height is more similar,
	# map_type: 0 = normal fractal, 1 = binary, 2 = 3 cost types
	def newMapFractal(self, maxCost, bump_factor, uniformity, map_type):
		side_length = 1
		num_its = 0
		s = 0
		cost_method_flag = map_type # 0 = normal fractal, 1 = binary, 2 = 3 cost types

		rand_range = 1.0

		# find out the size of the new fractal
		while side_length < max(self.y_range, self.x_range) - 1:
			side_length = side_length * 2
			num_its = num_its + 1

		s = side_length
		side_length = side_length + 1

		fractal_map = {}

		for y in range(0, s+1):
			for x in range(0, s+1):
				fractal_map[(y, x)] = -500000

		# initialize corners
		fractal_map[(0, 0)] = (0.5 - round(random.random(), 4))*2*rand_range
		fractal_map[(0, s)] = (0.5 - round(random.random(), 4))*2*rand_range
		fractal_map[(s, 0)] = (0.5 - round(random.random(), 4))*2*rand_range
		fractal_map[(s, s)] = (0.5 - round(random.random(), 4))*2*rand_range

		print(fractal_map)

		s_max = 2
		same_count = 0
		while s_max <= s:
			if same_count < uniformity*num_its:
				pass
			else:
				rand_range = rand_range*bump_factor

			# x case, we want only the odd multiples
			for r in range(1, s_max, 2):
				this_r = r*s/s_max
				for c in range(1, s_max, 2):
					this_c = c*s/s_max

					# neighbors are s/s_max left and right and above and below
					if same_count < (uniformity*num_its):
						fractal_map[(this_r, this_c)] = (0.5 - round(random.random(), 4))*2*rand_range
					else:
						na = fractal_map[(this_r-s/s_max, this_c-s/s_max)]
						nb = fractal_map[(this_r-s/s_max, this_c+s/s_max)]
						nc = fractal_map[(this_r+s/s_max, this_c-s/s_max)]
						nd = fractal_map[(this_r+s/s_max, this_c+s/s_max)]
						fractal_map[(this_r, this_c)] = (na + nb + nc + nd)/4 + (0.5 - round(random.random(), 4))*2*rand_range

			# x case, we want only the odd multiples
			for r in range(0, s_max+1):
				this_r = r*s/s_max
				for c in range(0, s_max+1):
					this_c = c*s/s_max

					# if this place in the map has a value already
					if fractal_map[(this_r, this_c)] != -500000:
						continue

					if same_count < (uniformity*num_its):
						fractal_map[(this_r, this_c)] = (0.5 - round(random.random(), 4))*2*rand_range
					else:
						nbrs = 4
						# neighbors are s/s_max left and right and above and below
						if this_r-s/s_max < 0: # no neighbor above
							na = 0 #fractal_map[this_r-s/s_max+s][this_c]
							nbrs = nbrs - 1
						else:
							na = fractal_map[(this_r-s/s_max, this_c)]

						if this_r+s/s_max > s: # no neighbor below
							nb = 0 #fractal_map[this_r+s/s_max-s][this_c];
							nbrs = nbrs - 1
						else:
							nb = fractal_map[(this_r+s/s_max, this_c)]

						if this_c-s/s_max < 0: # no neighbor to the left
							nc = 0 #fractal_map[this_r][this_c-s/s_max+s];
							nbrs = nbrs - 1
						else:
							nc = fractal_map[(this_r, this_c-s/s_max)]

						if this_c+s/s_max > s: # no neighbor to the right
							nd = 0 #fractal_map[this_r][this_c+s/s_max-s];
							nbrs = nbrs - 1
						else:
							nd = fractal_map[(this_r, this_c+s/s_max)]

						fractal_map[(this_r, this_c)] = (na + nb + nc + nd)/nbrs + (0.5 - round(random.random(), 4))*2*rand_range

			if same_count < uniformity * num_its:
				same_count = same_count + 1

			print(s_max)
			s_max = s_max*2

		# read fractal_map into map
		min_val = fractal_map[(0, 0)]
		max_val = fractal_map[(0, 0)]

		_map = {}

		for y in range(0, s+1):
			for x in range(0, s+1):
				_map[(y, x)] = -500000

		for r in range(0, self.y_range):
			for c in range(0, self.x_range):
				_map[(r, c)] = fractal_map[(r, c)]

				if _map[(r, c)] < min_val:
					min_val = _map[(r, c)]

				if _map[(r, c)] > max_val:
					max_val = _map[(r, c)]

		for r in range(0, self.y_range):
			for c in range(0, self.x_range):
				if cost_method_flag == 0: # normal fractal
					self.costs[(r, c)] = round(1 + (_map[(r, c)] - min_val)/(max_val - min_val)*(maxCost-1), 2)
				elif cost_method_flag == 1: # binary
					_map[(r, c)] = (_map[(r, c)] - min_val)/(max_val - min_val)*maxCost
					if _map[(r, c)] > .5*maxCost:
						self.costs[(r, c)] = maxCost
					else:
						self.costs[(r, c)] = 1
				elif cost_method_flag == 2: # 3 cost types
					 _map[(r, c)] = (_map[(r, c)] - min_val)/(max_val - min_val)*maxCost

					 if _map[(r, c)] > .67*maxCost:
						 self.costs[(r, c)] = maxCost
					 elif _map[(r, c)] > .33*maxCost:
						 self.costs[(r, c)] = 5
					 else:
						 self.costs[(r, c)] = 1

	def observe(self, s):
		result = []
		for u in self.motions:
			s_next = tuple([s[i] + u[i] for i in range(2)])
			if s_next not in self.border:
				result.append((s_next, self.costs[s_next]))
		return result

	def getCost(self, s):
		return self.costs[s]

	def test_obstacle(self, s):
		if s in self.obs:
			return True
		else:
			return False

	def get_neighbor(self, s):
		nei_list = set()
		for u in self.motions:
			s_next = tuple([s[i] + u[i] for i in range(2)])
			if s_next not in self.obs:
				nei_list.add(s_next)
		return nei_list

	def get_all_obstacles(self, s):
		obstacle_list = set()
		for u in [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]:
			s_next = tuple([s[i] + u[i] for i in range(2)])
			if s_next in self.obs:
				obstacle_list.add(s_next)
		return obstacle_list

	def get_obstacles(self, s):
		obstacle_list = set()
		for u in self.motions:
			s_next = tuple([s[i] + u[i] for i in range(2)])
			if s_next in self.obs:
				obstacle_list.add(s_next)
		return obstacle_list

	def update_obs(self, obs):
		self.obs = obs

	def getBorder(self):
		return self.border

	def print_env(self, s_start, s_goal):
		for i in range(-1, self.y_range+1):
			for j in range(-1, self.x_range+1):
				if (i, j) in self.obs:
					print("-1,", end='')
				elif (i, j) == s_start:
					print("A,", end='')
				elif (i, j) == s_goal:
					print("G,", end='')
				else:
					print(str(self.costs[(i, j)]) + ",", end='')
			print("\n")

	# these are the obstacles
	def obs_map(self):
		#x = self.x_range
		#y = self.y_range
		obs = set()

		obs.add((0, 4))
		obs.add((1, 2))
		obs.add((1, 3))
		obs.add((1, 4))
		obs.add((2, 4))
		#obs.add((3, 5))

		#obs.add((0, 12))
		obs.add((2, 12))
		obs.add((1, 12))
		obs.add((3, 12))
		obs.add((1, 11))

		#obs.add((3, 11))

		return obs

	# this is the border of the map
	def border_map(self):
		"""
		Initialize obstacles' positions
		:return: map of obstacles
		"""

		"""x = self.x_range
		y = self.y_range
		obs = set()

		for i in range(x):
			obs.add((i, 0))
		for i in range(x):
			obs.add((i, y - 1))

		for i in range(y):
			obs.add((0, i))
		for i in range(y):
			obs.add((x - 1, i))

		return obs"""

		x = self.x_range
		y = self.y_range
		obs = set()

		for i in range(x+2):
			obs.add((-1, i-1))

		for i in range(x+2):
			obs.add((self.y_range, i-1))

		for i in range(y):
			obs.add((i, -1))

		for i in range(y):
			obs.add((i, self.x_range))

		return obs
