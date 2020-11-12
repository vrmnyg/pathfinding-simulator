"""
FieldD algorithm
http://robots.stanford.edu/isrr-papers/draft/stentz.pdf
"""

import math
import utils.queue as queue
from algorithms.abstract_algorithm import AbstractAlgorithm

# globals
SMALL = 0.000001

# represent a path from one edge of a cell to another edge
# used when extracting path
class EdgePath:
	def __init__(self, y, x):
		self.cell_y = y
		self.cell_x = x
		self.length = 0
		self.y = []
		self.x = []

		self.y.insert(0, 0)
		self.x.insert(0, 0)
		self.y.insert(1, 0)
		self.x.insert(1, 0)
		self.y.insert(2, 0)
		self.x.insert(2, 0)
		# total path cost
		self.g = 0
		# total path cost - g cost (g_1 - g_2)
		self.local_g = 0
		# g cost to edge of the cell
		self.g_to_edge = 0

	def __lt__(self, other):
		return self.g < other.g

	def __le__(self, other):
		return self.g <= other.g

	def __eq__(self, other):
		return self.g == other.g

	def __ne__(self, other):
		return self.g != other.g

	def __gt__(self, other):
		return self.g > other.g

	def __ge__(self, other):
		return self.g >= other.g

class FieldD(AbstractAlgorithm):
	def __init__(self, s_start, s_goal, heuristic_type, env, y, x, m, obs, border, c):
		super().__init__(s_goal, heuristic_type, y, x, m, obs, border, c)

		self.s_start = s_start
		self.o_start = s_start

		self.Env = env  # class Env

		# dictionaries
		self.g, self.rhs = {}, {}

		self.OPEN = queue.QueuePrior()  # OPEN set
		self.PARENT = dict()

		self.CPHEAP = queue.QueuePrior()

		self.visited = []  # order of visited nodes in planning
		#self.path = []

		# set all rhs and g values to infinity
		for i in range(0, self.y):
			for j in range(0, self.x):
				self.rhs[(i, j)] = math.inf
				self.g[(i, j)] = math.inf

		# start searching from goal by setting goal rhs to zero
		self.rhs[self.s_goal] = 0.0

		heuristic = self.h(self.s_goal, self.s_start)

		key_1 = min(self.g[self.s_goal], self.rhs[self.s_goal]) + heuristic
		key_2 = min(self.g[self.s_goal], self.rhs[self.s_goal])

		self.OPEN.put(self.s_goal, (key_1, key_2))

		# round to what decimals
		self.precision = 6

	# recalculating route sometimes break g-table, try to fix it
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

	# key calculation from paper
	def calculateKey(self, s):
		heuristic = self.h(s, self.s_start)

		# round(v_s, 5)
		key_1 = round(min(self.g[s], self.rhs[s]) + heuristic, self.precision)
		key_2 = round(min(self.g[s], self.rhs[s]), self.precision)
		return (key_1, key_2)

	# returns the clockwise cell of s - sn
	def cknbr(self, s, sn):
		new_x = s[1]
		new_y = s[0]
		delta_x = new_x - sn[1]
		delta_y = new_y - sn[0]

		if delta_y == -1:
			if delta_x == 1:
				new_y += 1
			else:
				new_x += 1
		elif delta_y == 1:
			if delta_x == -1:
				new_y -= 1
			else:
				new_x -= 1
		elif delta_y == 0:
			if delta_x == -1:
				new_y -= 1
			else:
				new_y += 1

		sc = (new_y, new_x)
		if sc in self.border:
			return None

		return sc

	# returns the counter clockwise cell of s - sn
	def ccknbr(self, s, sn):
		new_x = s[1]
		new_y = s[0]
		delta_x = new_x - sn[1]
		delta_y = new_y - sn[0]

		if delta_y == 1:
			if delta_x == 1:
				new_y -= 1
			else:
				new_x += 1
		elif delta_y == -1:
			if delta_x == -1:
				new_y += 1
			else:
				new_x -= 1
		elif delta_y == 0:
			if delta_x == 1:
				new_y -= 1
			else:
				new_y += 1

		scc = (new_y, new_x)
		if scc in self.border:
			return None
		return scc

	# computeCost(sn, s, sc)
	# this is used in computing the cost of g-values in g-table
	def computeCost(self, s, s_a, s_b):
		s_1 = s_a
		s_2 = s_b

		if s[1] == s_b[1] or s[0] == s_b[0]: # then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
			s_1 = s_b
			s_2 = s_a
		#else
		#    s_1 = s_a
		#    s_2 = s_b

		traversalY = min(s[0], s_1[0], s_2[0])
		traversalX = min(s[1], s_1[1], s_2[1])
		c = self.costs[(traversalY, traversalX)]
		b = c

		# b is traversal cost of map cell with corners s and s_1, not s_2
		b_y = min(s[0], s_1[0])
		b_x = min(s[1], s_1[1])

		# the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
		if s_1[0] < s_2[0]: # then c is actually below b
			b_y -= 1
		elif s_1[1] < s_2[1]: # then c is actually to the right of b
			b_x -= 1

		# need to take care of edge cases (WIDTH or HEIGHT or -1 or -1)
		if not (b_y == self.y or b_x == self.x or b_y == -1 or b_x == -1):
			b = self.costs[(b_y,  b_x)]

		if min(c,b) >= math.inf:
			v_s = math.inf
		elif self.g[s_1] <= self.g[s_2]:
			v_s = min(c,b) + self.g[s_1]
		else:
			f = self.g[s_1] - self.g[s_2]
			if f <= b:
				if c <= f:
					v_s = c*math.sqrt(2) + self.g[s_2]
				else:
					y = min(f/math.sqrt(c*c - f*f), 1.0)
					v_s = (c*math.sqrt(1+y*y)) + (f*(1-y)) + self.g[s_2]
			else:
				if c <= b:
					v_s = c*math.sqrt(2) + self.g[s_2]
				else:
					x = 1 - min(b/math.sqrt(c*c - b*b), 1.0)
					v_s = (c*math.sqrt(1+((1-x)*(1-x)))) + (b*x) + self.g[s_2]
		return round(v_s, self.precision)

	def updateNode(self, sn):
		if self.g[sn] != self.rhs[sn]:
			if self.OPEN.exists(sn):
				self.OPEN.update(sn, self.calculateKey(sn))
			else:
				self.OPEN.put(sn, self.calculateKey(sn))

	# this method fills the g-table with appropriate values
	# these are used to navigate the map
	def computeShortestPath(self):
		CLOSED = []  # CLOSED set

		while self.OPEN.peek()[0] < self.calculateKey(self.s_start) or self.rhs[self.s_start] != self.g[self.s_start]:

			key = self.OPEN.peek()
			s = self.OPEN.get()
			CLOSED.append(s)

			if self.g[s] > self.rhs[s]:
				self.g[s] = self.rhs[s]

				#self.get_neighbor(s)
				for sn in self.get_neighbor(s):
					scc = self.ccknbr(s, sn)
					if scc != None:
						computedCost = self.computeCost(sn, s, scc)

						if self.rhs[sn] > computedCost:
							self.rhs[sn] = computedCost
							self.PARENT[sn] = s

					sc = self.cknbr(s, sn)
					if sc != None:
						computedCost = self.computeCost(sn, s, sc)

						if self.rhs[sn] > computedCost:
							self.rhs[sn] = computedCost
							self.PARENT[sn] = s

					self.updateNode(sn)
			else:
				self.g[s] = math.inf

				for sn in self.get_neighbor(s):
					self.UpdateState(sn)
				self.UpdateState(s)

		self.visited.append(CLOSED)

	"""This does majority of computations. It computes the minimum cost from a
	 point (y_agent, x_agent) inside cell c to the edge (of c) that connects
	 nodes s_1 and s_2. The node s is also located on a corner of c, and defines a
	 neighboring cell b that can also be used en-route to the edge (s_a s_b).
	 Assuming that s is the origin of a coordinate system where the y axis starts
	 at node s and goes along the edge between c and b, and the x axis starts at s
	 and goes along the other edge of c, and that side lengths of c are 1,
	 (y_agent, x_agent) is the location of the agent. this returns an EdgePath
	 containing relevant movement info"""
	def computePointCostToEdge(self, y_agent, x_agent, s, s_a, s_b, cell_y, cell_x):
		subPaths = []
		# there are three possible cases
		for j in range(3):
			subPaths.insert(j,  EdgePath(cell_y, cell_x))
		# case 0
		subPath = subPaths[0]

		if(s[1] == s_b[1] or s[0] == s_b[0]): # then s_b is horizontal or vertical neighbor of s, thus, s_a is a diagonal neighbor of s
			s_1 = s_b
			s_2 = s_a
		elif(s[1] == s_b[1] and s[0] == s_b[0]):
			print(" This should never happen 0\n")
			exit(0)
		else:
			s_1 = s_a
			s_2 = s_b

		traversalY = min(s[0], s_1[0], s_2[0])
		traversalX = min(s[1], s_1[1], s_2[1])
		c = self.costs[(traversalY, traversalX)]
		b = math.inf

		# b is traversal cost of map cell with corners s and s_1, not s_2
		b_y = min(s[0], s_1[0])
		b_x = min(s[1], s_1[1])

		# the above assumes that either c is above b or c is to the left of b, we need to modify things if this is not the case
		if s_1[0] < s_2[0]: # then c is actually below b
			b_y -= 1
		elif s_1[1] < s_2[1]: # then c is actually to the right of b
			b_x -= 1

		# need to take care of edge cases (WIDTH or HEIGHT or -1 or -1)
		if not (b_y == self.y or b_x == self.x or b_y == -1 or b_x == -1):
			b = self.costs[(b_y,  b_x)]

		# remember the original node values, because we will modify them locally if necessary
		# but want to leave them the same after we are done
		old_s_1_g = self.g[s_1]
		old_s_2_g = self.g[s_2]

		if self.g[s_1] == math.inf and self.g[s_2] == math.inf:
			# put temporary large values because math.inf - math.inf = nan, not zero
			# this leads to ZeroDivisionError in c/(self.g[s_1] - self.g[s_2])
			#self.g[s_1] = 100000000
			#self.g[s_2] = 100000000
			#print("INF and INF")
			pass
		elif self.g[s_1] > self.g[s_2] + c: # to avoid getting imaginary answer, need to give this an appropriate g based on s_2
			self.g[s_1] = self.g[s_2] + c
		elif self.g[s_2] > self.g[s_1] + c: # to avoid getting imaginary answer, need to give this an appropriate g based on s_1
			self.g[s_2] = self.g[s_1] + c

		# nan case (inf and inf), no movement
		if math.isnan(abs(self.g[s_1] - self.g[s_2])):
			subPath.length = 0
			return subPaths

		# inf case (other is inf), no movement
		if abs(self.g[s_1] - self.g[s_2]) == math.inf:
			subPath.length = 0
			return subPaths

		# if c is inf, no good path is possible at this point
		if c == math.inf:
			subPath.length = 0
			return subPaths

		# the point is on the edge we are trying to get to # no movement required, already there
		if(y_agent == 1): # then the point is on the edge that we are trying to get to, nothing will be better than staying at that point
			subPath.length = 0
			self.g[s_1]	= old_s_1_g
			self.g[s_2] = old_s_2_g
			# subPaths[0] = subPath
			return subPaths

		# the equation does not work, but we know that the best option is to go straight at the minimum of g1 or g2
		if abs(abs(self.g[s_1] - self.g[s_2]) - c) < SMALL:
			# don't use equations, assume straight to g_1 or g_2 is the best route
			if self.g[s_1] < self.g[s_2]:
				subPath.x[0] = 0
				subPath.local_g = round(c*math.sqrt(((x_agent)*(x_agent)) + ((1-y_agent)*(1-y_agent))), self.precision)
				subPath.g = round(subPath.local_g + self.g[s_1], self.precision)
			elif self.g[s_2] < self.g[s_1]:
				subPath.x[0] = 1
				subPath.local_g = round(c*math.sqrt(((1-x_agent)*(1-x_agent)) + ((1-y_agent)*(1-y_agent))), self.precision)
				subPath.g = round(subPath.local_g + self.g[s_2], self.precision)
			else:
				exit(0)
			subPath.y[0] = 1
			subPath.g_to_edge = subPath.local_g
			subPath.length = 1
		else:
			# equation used, calculate best x pos to go to
			c_temp = round(math.sqrt(((1-y_agent)*(1-y_agent))/(((c/(self.g[s_1] - self.g[s_2]))*(c/(self.g[s_1] - self.g[s_2])))-1)), self.precision)

			x_1 = x_agent + c_temp
			x_2 = x_agent - c_temp

			x_1 = round(max(min(x_1, 1.0), 0.0), self.precision)
			x_2 = round(max(min(x_2, 1.0), 0.0), self.precision)

			g_1 = round(c*math.sqrt(((1-y_agent)*(1-y_agent))+((x_1-x_agent)*(x_1-x_agent))) + x_1*self.g[s_2] + (1-x_1)*self.g[s_1], self.precision)
			g_2 = round(c*math.sqrt(((1-y_agent)*(1-y_agent))+((x_2-x_agent)*(x_2-x_agent))) + x_2*self.g[s_2] + (1-x_2)*self.g[s_1], self.precision)

			if g_1 < g_2:
				subPath.x[0] = x_1
				subPath.local_g = round(c*math.sqrt(((1-y_agent)*(1-y_agent))+((x_1-x_agent)*(x_1-x_agent))), self.precision)
				subPath.g = round(subPath.local_g + x_1*self.g[s_2] + (1-x_1)*self.g[s_1], self.precision)
			else:
				subPath.x[0] = x_2
				subPath.local_g = round(c*math.sqrt(((1-y_agent)*(1-y_agent))+((x_2-x_agent)*(x_2-x_agent))), self.precision)
				subPath.g = round(subPath.local_g + x_2*self.g[s_2] + (1-x_2)*self.g[s_1], self.precision)
			subPath.y[0] = 1
			subPath.g_to_edge = subPath.local_g
			subPath.length = 1

		#cases 1 and 2 apply only if b < c
		if b < c:
			subPath = subPaths[1]

			# option 1, the path goes to b and then along b and then across c to s_2. This can only happen if s_2->g is less than s_1->g
			if self.g[s_2] < self.g[s_1]:
				c_temp = round(math.sqrt(((1+x_agent)*(1+x_agent))/(((c*c)/(b*b))-1)), self.precision)
				ybar_1 = 1 + c_temp
				ybar_2 = 1 - c_temp

				ybar_1 = max(min(ybar_1, 1.0), 0.0)
				ybar_2 = max(min(ybar_2, 1.0), 0.0)

				g_1 = round(b*(abs(ybar_1-y_agent)) + c*math.sqrt(((1-ybar_1)*(1-ybar_1))+((1+x_agent)*(1+x_agent))), self.precision) + self.g[s_2]
				g_2 = round(b*(abs(ybar_2-y_agent)) + c*math.sqrt(((1-ybar_2)*(1-ybar_2))+((1+x_agent)*(1+x_agent))), self.precision) + self.g[s_2]

				#if(min(g_1, g_2) < subPaths[0].g):
				if True:
					subPath.x[0] = 0
					subPath.x[1] = 0

					# this path will eventually go to s_2
					if g_1 < g_2:
						subPath.y[0] = round((((1-ybar_1)*x_agent)/(1+x_agent)) + y_agent, self.precision)
						subPath.y[1] = round(1 - (1-ybar_1)/(1+x_agent), self.precision)
						diagonal_g = round(c*math.sqrt(((1-ybar_1)*(1-ybar_1))+((1+x_agent)*(1+x_agent))), self.precision)
						straight_g = round(b*(abs(ybar_1-y_agent)), self.precision)
					else: # g_2 <= g_1
						subPath.y[0] = round((((1-ybar_2)*x_agent)/(1+x_agent)) + y_agent, self.precision)
						subPath.y[1] = round(1 - (1-ybar_2)/(1+x_agent), self.precision)
						diagonal_g = round(c*math.sqrt(((1-ybar_2)*(1-ybar_2))+((1+x_agent)*(1+x_agent))), self.precision)
						straight_g = round(b*(abs(ybar_2-y_agent)), self.precision)

					# then the first point should be ignored because (subPath->y[0] == y_agent && subPath->x[0] == x_agent), also subPath->g_to_edge == 0
					# this means that the agent does not have to move to b edge, it is already there
					if x_agent == 0:
						subPath.y[0] = subPath.y[1]
						subPath.x[0] = subPath.x[1]
						subPath.x[1] = 1
						subPath.y[1] = 1
						subPath.g_to_edge = straight_g
						subPath.length = 2
					# all points are used, agent has to move to b edge and move along it
					else:
						subPath.x[2] = 1
						subPath.y[2] = 1
						subPath.g_to_edge = round(diagonal_g*x_agent/(1+x_agent), self.precision)
						subPath.length = 3
					subPath.local_g = round(diagonal_g + straight_g, self.precision)
					subPath.g = subPath.local_g + self.g[s_2]

			#subPaths[1] = subPath
			# option 2, the path goes to b and then along b to s_1. This may be better even if s_2->g is less than s_1->g
			subPath = subPaths[2]

			c_temp = round(math.sqrt((x_agent*x_agent)/(((c*c)/(b*b))-1)), self.precision)
			ybar_1 = y_agent + c_temp
			ybar_2 = y_agent - c_temp

			ybar_1 = round(max(min(ybar_1, 1.0), 0.0), self.precision)
			ybar_2 = round(max(min(ybar_2, 1.0), 0.0), self.precision)

			g_1 = round(b*(1-ybar_1) + c*math.sqrt(((ybar_1-y_agent)*(ybar_1-y_agent))+(x_agent*x_agent)), self.precision) + self.g[s_1]
			g_2 = round(b*(1-ybar_2) + c*math.sqrt(((ybar_2-y_agent)*(ybar_2-y_agent))+(x_agent*x_agent)), self.precision) + self.g[s_1]

			# need to compare to old g value to see if this is any better, if it is, then this path will eventually go to s_1
			#if min(g_1, g_2) < subPaths[0].g:
			if True:
				subPath.x[0] = 0

				if g_1 < g_2:
					subPath.y[0] = ybar_1
					subPath.g_to_edge = round(c*math.sqrt(((ybar_1-y_agent)*(ybar_1-y_agent))+(x_agent*x_agent)), self.precision)
					subPath.local_g = round(subPath.g_to_edge + b*(1-ybar_1), self.precision)
				else: # g_2 <= g_1
					subPath.y[0] = ybar_2
					subPath.g_to_edge = round(c*math.sqrt(((ybar_2-y_agent)*(ybar_2-y_agent))+(x_agent*x_agent)), self.precision)
					subPath.local_g = round(subPath.g_to_edge + b*(1-ybar_2), self.precision)

				if x_agent == 0: # then the first point should be ignored because (subPath->y[0] == y_agent && subPath->x[0] == x_agent), also subPath->g_to_edge == 0
					subPath.x[0] = 0
					subPath.y[0] = 1
					subPath.g_to_edge = subPath.local_g
					subPath.length = 1
				else: # all points are used
					subPath.x[1] = 0
					subPath.y[1] = 1
					subPath.length = 2
				subPath.g = round(subPath.local_g + self.g[s_1], self.precision)

		self.g[s_1] = old_s_1_g
		self.g[s_2] = old_s_2_g
		return subPaths

	"""there are 4 possible edges to go to (defined by s_a and s_b), and
	   all edges can have 2 different s (point of origin),
	   therefore we need to call computePointCostToEdge 8 times"""
	def computeLocalPointCost(self, cell_y, cell_x, y, x):
		cell = (cell_y, cell_x)
		cell_r = tuple(map(lambda i, j: i + j, cell, (0,1)))
		cell_c = tuple(map(lambda i, j: i + j, cell, (1,1)))
		cell_d = tuple(map(lambda i, j: i + j, cell, (1,0)))

		edges = [(cell, cell_r), (cell_r, cell_c), (cell_c, cell_d), (cell_d, cell)]
		bot_pos_sc = [(1 - y, 1 - x), (x, 1 - y), (y, x), (1 - x, y)]
		bot_pos_scc = [(1 - y, x), (x, y), (y, 1 - x), (1 - x, 1 - y)]

		for ec in range(len(edges)):
			s_a = edges[ec][0]
			s_b = edges[ec][1]

			sc = self.cknbr(s_b, s_a)
			if sc != None:
				# (y_agent, x_agent) is the robots position in relation to point of origin (s)
				# need to figure out it in relation to sc
				paths_sc = self.computePointCostToEdge(bot_pos_sc[ec][0], bot_pos_sc[ec][1], sc, s_a, s_b, cell_y, cell_x)

				# self.computePointCostToEdge returns 3 cases, go through them
				# return is in relative path, switch to absolute path
				for j in range(3):
					subPath = paths_sc[j]

					if ec == 0: # s_a is in the upper left
						for i in range(subPath.length):
							t_x = 1 - subPath.x[i]
							t_y = 1 - subPath.y[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 1: # s_a is in the upper right
						for i in range(subPath.length):
							t_x = subPath.y[i]
							t_y = 1 - subPath.x[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 2: # s_a is in the lower right
						for i in range(subPath.length):
							t_x = subPath.x[i]
							t_y = subPath.y[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 3: # s_a is in the lower left
						for i in range(subPath.length):
							t_x = 1 - subPath.y[i]
							t_y = subPath.x[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y

					if subPath.length > 0 and not math.isnan(subPath.g):
						self.CPHEAP.put(subPath, subPath.g)

			scc = self.ccknbr(s_a, s_b)
			if scc != None:
				# (y_agent, x_agent) is the robots position in relation to point of origin (s)
				# need to figure out it in relation to scc
				paths_scc = self.computePointCostToEdge(bot_pos_scc[ec][0], bot_pos_scc[ec][1], scc, s_a, s_b, cell_y, cell_x)

				for j in range(3):
					subPath = paths_scc[j]

					if ec == 0: # s_a is in the upper left
						for i in range(subPath.length):
							t_x = subPath.x[i]
							t_y = 1 - subPath.y[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 1: # s_a is in the upper right
						for i in range(subPath.length):
							t_x = subPath.y[i]
							t_y = subPath.x[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 2: # s_a is in the lower right
						for i in range(subPath.length):
							t_x = 1 - subPath.x[i]
							t_y = subPath.y[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y
					elif ec == 3: # s_a is in the lower left
						for i in range(subPath.length):
							t_x = 1 - subPath.y[i]
							t_y = 1 - subPath.x[i]
							subPath.x[i] = t_x
							subPath.y[i] = t_y

					# add this EdgePath to the EdgePath heap
					if subPath.length > 0 and not math.isnan(subPath.g):
						self.CPHEAP.put(subPath, subPath.g)

	# find out the neighboring cells of current position and call computeLocalPointCost
	def computeAllLocalPointCosts(self, current_pos):
		float_pos_y = float(current_pos[0])
		float_pos_x = float(current_pos[1])
		int_pos_y = int(current_pos[0])
		int_pos_x = int(current_pos[1])

		if float_pos_x == float(int_pos_x):
			x_f = [i for i in [int_pos_x - 1, int_pos_x] if (i >= 0) and (i < (self.x - 1))]
		else:
			x_f = [int_pos_x]

		if float_pos_y == float(int_pos_y):
			y_f = [i for i in [int_pos_y - 1, int_pos_y] if (i >= 0) and (i < (self.y - 1))]
		else:
			y_f = [int_pos_y]

		# these are the neighboring cells of current_pos, call
		# computeLocalPointCost for each to figure out shortest path through them
		# (agent_y, agent_x) is agents current relative position (cell)
		cells = [(i,j) for i in y_f for j in x_f]

		for cell in cells:
			if cell[0] == int_pos_y - 1:
				agent_y = 1.0
			else:
				agent_y = float_pos_y - float(int_pos_y)

			if cell[1] == int_pos_x - 1:
				agent_x = 1.0 # bot position relative to path_cell_x
			else:
				agent_x = float_pos_x - float(int_pos_x)

			self.computeLocalPointCost(cell[0], cell[1], agent_y, agent_x)

	# resets all to starting values
	def reset(self):
		self.g, self.rhs = {}, {}

		self.OPEN = queue.QueuePrior()  # OPEN set
		self.PARENT = dict()

		self.CPHEAP = queue.QueuePrior()

		#self.visited = []  # order of visited nodes in planning
		#self.path = []

		# set all rhs and g values to infinity
		for i in range(0, self.y):
			for j in range(0, self.x):
				self.rhs[(i, j)] = math.inf
				self.g[(i, j)] = math.inf

		# start searching from goal by setting goal rhs to zero
		self.rhs[self.s_goal] = 0.0

		heuristic = self.h(self.s_goal, self.s_start)

		key_1 = min(self.g[self.s_goal], self.rhs[self.s_goal]) + heuristic
		key_2 = min(self.g[self.s_goal], self.rhs[self.s_goal])

		self.OPEN.put(self.s_goal, (key_1, key_2))

	# print
	def print_env(self):
		for i in range(-1, self.y+1):
			for j in range(-1, self.x+1):
				if (i, j) in self.border:
					print("#|", end='')
				else:
					print(str(self.costs[(i, j)]) + ",", end='')
			print("\n")

	# find the best path from start to goal, return True if path is not valid, try to fix it
	def extract_path(self):
		orig_pos = self.s_start
		current_pos = self.s_start
		robotsPath = []
		pathCosts = []
		totalPathCost = 0.0

		while(current_pos != self.s_goal):

			self.computeAllLocalPointCosts(current_pos)

			try:
				topEdgePath = self.CPHEAP.get()
			except IndexError as ie:
				raise

			# # TODO: special case
			#if topEdgePath.length != 1:
			#	wait = input("topEdgePath.length case, TODO")

			pos_x = round(float(topEdgePath.cell_x + topEdgePath.x[0]), self.precision)
			pos_y = round(float(topEdgePath.cell_y + topEdgePath.y[0]), self.precision)

			if (pos_y, pos_x) == current_pos:
				print("ER - 2")
				wait = input("pos == pos")
				continue

			robotsPath.append((pos_y, pos_x))

			# test for valid path, if not valid raturn True, and not valid parth of path
			# path and cost are empty because no valid path was found
			if len(robotsPath) > 3 and robotsPath[-1] == robotsPath[-3]:
				looping_s = robotsPath[-1]
				del robotsPath[-1]
				self.CPHEAP = queue.QueuePrior()
				return True, looping_s, None, None

			totalPathCost = totalPathCost + topEdgePath.g_to_edge
			pathCosts.append(topEdgePath.g_to_edge)

			self.CPHEAP = queue.QueuePrior()
			current_pos = (pos_y, pos_x)

		# path is valid, no problem part of path, return path and cost
		return False, None, robotsPath, pathCosts

	def findMinRhs(self, s):
		minRhs = math.inf
		thisC = math.inf
		s_min = None

		for sn in self.get_neighbor(s):
			scc = self.ccknbr(sn, s)
			if scc != None:
				thisC = self.computeCost(s, sn, scc)
				if thisC < minRhs:
					minRhs = thisC
					if self.rhs[sn] < self.rhs[scc]:
						s_min = sn
					else:
						s_min = scc
			sc = self.cknbr(sn, s)
			if sc != None:
				thisC = self.computeCost(s, sn, sc)
				if thisC < minRhs:
					minRhs = thisC
					if self.rhs[sn] < self.rhs[sc]:
						s_min = sn
					else:
						s_min = sc
		return s_min, minRhs

	def UpdateState(self, s):
		try:
			parent = self.PARENT[s]
		except KeyError:
			parent = None

		if parent != None:
			s_min, minRhs = self.findMinRhs(s)
			self.rhs[s] = minRhs
			self.PARENT[s] = s_min
			self.updateNode(s)

	# this changes g-table if changes in map are found
	def ChangeEdgeCosts(self, s):
		temp = []
		for i in s:
			if i not in self.obs:
				self.obs.add(i)
			temp.append(i)
			self.g[i] = math.inf
			self.rhs[i] = math.inf
			if self.OPEN.exists(i):
				self.OPEN.remove(i)

		for j in temp:
			if j not in self.border:
				for sn in self.get_neighbor(j):
					self.UpdateState(sn)

	# this is the main loop of the algorithm
	def run(self, s_start):
		self.s_start = s_start
		# fill the g-table
		self.computeShortestPath()

		# use the g-table to calculate path from start to goal, fix broken g-table if necessary
		loop, sl, self.route, pathCosts = self.extract_path()
		while loop:
			self.fix(sl)
			self.computeShortestPath()
			loop, sl, self.route, pathCosts = self.extract_path()

		yield self.route, pathCosts
