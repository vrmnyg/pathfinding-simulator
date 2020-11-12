"""
FieldD navigator
This class contains navigation logic for fieldD algorithm
"""
import operator
import math
import queue
from navigators.abstract_navigator import AbstractNavigator
from algorithms.fieldD import FieldD

class FieldDNavigator(AbstractNavigator):
	def __init__(self, env, s_start, s_goal):
		self.s_goal = s_goal
		#self.Env = env  # class Env
		#self.position = s_start
		self.o_start = s_start

		#self.border = self.Env.getBorder()
		#self.route = []
		#self.path = []
		#self.totalPathCost = 0

		# this saves all the movement costs in an array
		# can be removed if not used
		self.pathcosts = []

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
						#costs[(i, j)] = 1
					else:
						costs[(i, j)] = env.getCost((i, j))

		self.fieldd = FieldD(s_start,  s_goal, "euclidean", env, env.y_range, env.x_range, env.motions, obs, border, costs)

		#self.obs.add((2, 4))
		#self.costs[(2, 4)] = math.inf

		# some_list[-len(some_list)]
		# some_list[-1]

	# this method runs the searching and decides the movement of the agents
	# work in progress
	def run(self):
		self.position = self.o_start
		# locals
		#temp_costs = []
		temp_path = []
		new_observed_obs = []
		path = 0
		self.totalPathCost = 0
		self.pathcosts = []
		self.path = []
		self.route = []

		while self.s_goal != self.position:
			#start = timer()
			path_generator = self.fieldd.run(self.position)
			try:
				path, movementCosts = next(path_generator)
			except StopIteration:
				pass
			#finally:
			#	del path_generator

			temp_path.append(self.position)
			while len(path) > 0:
				step = path.pop(0)
				movementCost = movementCosts.pop(0)

				# with fieldD step can contain decimals, like e.g. (0.045, 4.0)
				# need to check if that step goes throug an obstacle.
				int_step = (int(step[0]), int(step[1]))
				is_obstacle = self.fieldd.Env.test_obstacle(int_step)

				#print(step)
				#print(int_step)
				#print(movementCost)
				#print(is_obstacle)
				#wait = input("step...")

				if is_obstacle:
					# agent is in a full int position and finds a blocking obstacle
					int_position = (int(self.position[0]), int(self.position[1]))
					if self.position == int_position:
						self.path.append(temp_path)
						break
					# this happens when agent hits an obstacle in a position between nodes
					# fieldD algorithm can not handle decimal starting positions like (2.54, 3.56)
					# must be full int e.g. (2, 3)
					# navigator must figure out which way to go and move there before calling fieldD
					# optimization can be done here!
					else:
						n = int_position
						print(self.position)
						print(int_position)
						print(self.fieldd.obs - self.fieldd.border)

						# all the neighbors of current position
						sn = self.fieldd.get_neighbor(int_position)

						# use the current moving direction to figure out which way to go
						if len(temp_path) > 1:
							# if positive, moving "up"
							vertical_movement = temp_path[-2][0] - temp_path[-1][0]
							# if positive, moving "right"
							horizontal_movement = temp_path[-1][1] - temp_path[-2][1]

							if horizontal_movement > 0:
								horizontal_movement = 1
							elif horizontal_movement < 0:
								horizontal_movement = -1
							else:
								horizontal_movement = 0

							if vertical_movement > 0:
								vertical_movement = -1
							elif vertical_movement < 0:
								vertical_movement = 1
							else:
								vertical_movement = 0

							movement = (vertical_movement, horizontal_movement)

							# figure out whic way is the obstacle
							obstacle_direction = tuple(map(operator.sub, int_step, int_position))
							#best_move = tuple(map(operator.sub, int_position, movement))
							#print(best_move)
							if obstacle_direction[1] == 0 and obstacle_direction[0] == 1:
								#print("obstacle is down")
								if movement[1] == 1:
									#print("moving right")
									if (int_position[0] + 1, int_position[1] + 1) in sn:
										n = (int_position[0] + 1, int_position[1] + 1)
									elif (int_position[0] + 1, int_position[1] - 1) in sn:
										n = (int_position[0] + 1, int_position[1] - 1)
									else:
										pass
										#wait = input("both candidates are obstacles!")
								elif movement[1] == -1:
									#print("moving left")
									if (int_position[0] + 1, int_position[1] - 1) in sn:
										n = (int_position[0] + 1, int_position[1] - 1)
									elif (int_position[0] + 1, int_position[1] + 1) in sn:
										n = (int_position[0] + 1, int_position[1] + 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								else:
									wait = input("error!")
							elif obstacle_direction[1] == 0 and obstacle_direction[0] == -1:
								print("obstacle is up")
								if movement[1] == 1:
									print("moving right")
									if (int_position[0] - 1, int_position[1] + 1) in sn:
										n = (int_position[0] - 1, int_position[1] + 1)
									elif (int_position[0] - 1, int_position[1] - 1) in sn:
										n = (int_position[0] - 1, int_position[1] - 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								elif movement[1] == -1:
									print("moving left")
									if (int_position[0] - 1, int_position[1] - 1) in sn:
										n = (int_position[0] - 1, int_position[1] - 1)
									elif (int_position[0] - 1, int_position[1] + 1) in sn:
										n = (int_position[0] - 1, int_position[1] + 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								else:
									wait = input("error!")
							elif obstacle_direction[1] == 1 and obstacle_direction[0] == 0:
								#print("obstacle is on the right")
								if movement[0] == 1:
									#print("moving down")
									if (int_position[0] + 1, int_position[1] + 1) in sn:
										n = (int_position[0] + 1, int_position[1] + 1)
									elif (int_position[0] - 1, int_position[1] + 1) in sn:
										n = (int_position[0] - 1, int_position[1] + 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								elif movement[0] == -1:
									#print("moving up")
									if (int_position[0] - 1, int_position[1] + 1) in sn:
										n = (int_position[0] - 1, int_position[1] + 1)
									elif (int_position[0] + 1, int_position[1] + 1) in sn:
										n = (int_position[0] + 1, int_position[1] + 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								else:
									wait = input("error!")
							elif obstacle_direction[1] == -1 and obstacle_direction[0] == 0:
								#print("obstacle is on the left")
								if movement[0] == 1:
									#print("moving down")
									if (int_position[0] + 1, int_position[1] - 1) in sn:
										n = (int_position[0] + 1, int_position[1]- 1)
									elif (int_position[0] - 1, int_position[1] - 1) in sn:
										n = (int_position[0] - 1, int_position[1] - 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								elif movement[0] == -1:
									#print("moving up")
									if (int_position[0] - 1, int_position[1] - 1) in sn:
										n = (int_position[0] - 1, int_position[1]- 1)
									elif (int_position[0] + 1, int_position[1] - 1) in sn:
										n = (int_position[0] + 1, int_position[1] - 1)
									else:
										#wait = input("both candidates are obstacles!")
										pass
								else:
									wait = input("error!")
							else:
								# not implemented TODO
								wait = input("diagonal obstacle...")

						# not moved yet, cant figure out which way to go from direction
						# maybe move to cheapest node?
						else:
							# not implemented TODO
							wait = input("len too small...figure out way some other way...")
						#print(n)
						#print("must move from " + str(self.position) + " to " + str(n))
						#wait = input("start counting...")

						# simplest possible move
						# n was not changed
						if n == int_position:
							from_position_to_goal_distance = math.sqrt(((self.position[0]-n[0])**2)+((self.position[1]-n[1])**2))

							if horizontal_movement == -1 or horizontal_movement == 1:
								#print("simple move, move horizontally")
								cost1 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1] - 1)]
								cost = min(cost1, cost2)
								#print(cost)
							elif vertical_movement == -1 or vertical_movement == 1:
								#print("simple move, move vertically")
								cost1 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0] - 1, int_position[1])]
								cost = min(cost1, cost2)
								#print(cost)
							else:
								wait = input("?")

							if cost == math.inf:
								wait = input("cost error, cost math.inf")

							self.position = n
							self.route.append(self.position)
							self.totalPathCost = self.totalPathCost + cost
							self.pathcosts.append(cost)

							observed = list(self.observe() - self.fieldd.obs)
							new_observed_obs.extend(observed)
							self.fieldd.obs = self.fieldd.obs | self.observe()
							#temp_costs.append(movementCost)
							temp_path.append(n)

							# keyerror when position has decimals, like (2,45, 2)
							try:
								int_position = (int(self.position[0]), int(self.position[1]))
								for o in self.fieldd.Env.observe(int_position):
									self.fieldd.costs[o[0]] = o[1]
							# no need to update costs when keyerror, only full (INT, INT) positions have costs
							except KeyError:
								pass
							self.path.append(temp_path)
							print(self.pathcosts)

							break

						y_diff = n[0] - self.position[0]
						x_diff = n[1] - self.position[1]
						ratio = y_diff/x_diff

						from_position_to_goal_distance = math.sqrt(((self.position[0]-n[0])**2)+((self.position[1]-n[1])**2))
						print(from_position_to_goal_distance)

						if abs(y_diff) > 1 and abs(x_diff) > 1:
							wait = input("?")
						# if diff is more than 1 path goes through multiple nodes
						# must calculate cost if they have different path costs
						# TODO
						# if costs are the same only distance matters
						# no need to calculate different costs
						elif abs(y_diff) > 1:
							#print("y direction crosses s")
							if y_diff < 0:
								y_cross = int_position[0]
								x = (y_cross - self.position[0] + ratio * self.position[1])/ratio
								middle_point = (y_cross, x)

								from_position_to_middle_distance = math.sqrt(((self.position[0]-middle_point[0])**2)+((self.position[1]-middle_point[1])**2))
								from_middle_to_goal_distance = math.sqrt(((n[0]-middle_point[0])**2)+((n[1]-middle_point[1])**2))

								cost1 = from_position_to_middle_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_middle_to_goal_distance * self.fieldd.costs[(int_position[0] - 1, int_position[1])]

								cost = cost1 + cost2
							elif y_diff > 0:
								y_cross = math.ceil(int_position[0])
								x = (y_cross - self.position[0] + ratio * self.position[1])/ratio
								middle_point = (y_cross, x)

								from_position_to_middle_distance = math.sqrt(((self.position[0]-middle_point[0])**2)+((self.position[1]-middle_point[1])**2))
								from_middle_to_goal_distance = math.sqrt(((n[0]-middle_point[0])**2)+((n[1]-middle_point[1])**2))

								cost1 = from_position_to_middle_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_middle_to_goal_distance * self.fieldd.costs[(int_position[0] + 1, int_position[1])]

								cost = cost1 + cost2
							else:
								wait = input("0")
						elif abs(x_diff) > 1:
							#print("x direction crosses s")
							if x_diff < 0:
								x_cross = int_position[1]
								y  =  ratio * (x_cross - self.position[1]) + self.position[0]
								middle_point = (y, x_cross)

								from_position_to_middle_distance = math.sqrt(((self.position[0]-middle_point[0])**2)+((self.position[1]-middle_point[1])**2))
								from_middle_to_goal_distance = math.sqrt(((n[0]-middle_point[0])**2)+((n[1]-middle_point[1])**2))

								cost1 = from_position_to_middle_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_middle_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1] - 1)]

								cost = cost1 + cost2
							elif x_diff > 0:
								x_cross = math.ceil(int_position[1])
								y  =  ratio * (x_cross - self.position[1]) + self.position[0]
								middle_point = (y, x_cross)

								from_position_to_middle_distance = math.sqrt(((self.position[0]-middle_point[0])**2)+((self.position[1]-middle_point[1])**2))
								from_middle_to_goal_distance = math.sqrt(((n[0]-middle_point[0])**2)+((n[1]-middle_point[1])**2))

								cost1 = from_position_to_middle_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_middle_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1] + 1)]

								cost = cost1 + cost2
							else:
								wait = input("0")
						else:
							#print("no crossing")
							from_position_to_goal_distance = math.sqrt(((self.position[0]-n[0])**2)+((self.position[1]-n[1])**2))

							if x_diff == 1:
								#print("moves right")
								cost = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1])]
							elif x_diff == -1:
								#print("moves left")
								cost = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1] - 1)]
							elif y_diff == 1:
								#print("moves down")
								cost = from_position_to_goal_distance * self.fieldd.costs[(int_position[0] + 1, int_position[1])]
							elif y_diff == -1:
								#print("moves up")
								cost = from_position_to_goal_distance * self.fieldd.costs[(int_position[0] - 1, int_position[1])]
							else:
								wait = input("no 1, error!")

						# simplest possible move
						# n was not changed or the new path cost is too high
						if n == int_position or cost == math.inf:
							n = int_position
							from_position_to_goal_distance = math.sqrt(((self.position[0]-n[0])**2)+((self.position[1]-n[1])**2))

							if horizontal_movement == -1 or horizontal_movement == 1:
								#print("simple move, move horizontally")
								cost1 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1] - 1)]
								cost = min(cost1, cost2)
								#print(cost)
							elif vertical_movement == -1 or vertical_movement == 1:
								#print("simple move, move vertically")
								cost1 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0], int_position[1])]
								cost2 = from_position_to_goal_distance * self.fieldd.costs[(int_position[0] - 1, int_position[1])]
								cost = min(cost1, cost2)
								#print(cost)
							else:
								wait = input("?")

							if cost == math.inf:
								wait = input("cost error, cost math.inf")

							self.position = n
							self.route.append(self.position)
							self.totalPathCost = self.totalPathCost + cost
							self.pathcosts.append(cost)

							observed = list(self.observe() - self.fieldd.obs)
							new_observed_obs.extend(observed)
							self.fieldd.obs = self.fieldd.obs | self.observe()
							#temp_costs.append(movementCost)
							temp_path.append(n)

							# keyerror when position has decimals, like (2,45, 2)
							try:
								# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
								int_position = (int(self.position[0]), int(self.position[1]))
								for o in self.fieldd.Env.observe(int_position):
									self.fieldd.costs[o[0]] = o[1]
							# no need to update costs when keyerror, only full (INT, INT) positions have costs
							except KeyError:
								pass
							self.path.append(temp_path)
							break

						self.position = n
						self.route.append(self.position)

						if cost == math.inf:
							print("cost == inf")
							wait = input("cost error, cost math.inf")
						self.totalPathCost = self.totalPathCost + cost
						self.pathcosts.append(cost)

						observed = list(self.observe() - self.fieldd.obs)
						new_observed_obs.extend(observed)
						self.fieldd.obs = self.fieldd.obs | self.observe()
						#temp_costs.append(movementCost)
						temp_path.append(n)

						# keyerror when position has decimals, like (2,45, 2)
						try:
							# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							int_position = (int(self.position[0]), int(self.position[1]))
							for o in self.fieldd.Env.observe(int_position):
								self.fieldd.costs[o[0]] = o[1]
						# no need to update costs when keyerror, only full (INT, INT) positions have costs
						except KeyError:
							pass

						self.path.append(temp_path)
						break
				else:
					self.position = step
					self.route.append(self.position)
					self.totalPathCost = self.totalPathCost + movementCost
					self.pathcosts.append(movementCost)

					observed = list(self.observe() - self.fieldd.obs)
					new_observed_obs.extend(observed)
					#wait = input("observed")

					self.fieldd.obs = self.fieldd.obs | self.observe()
					#temp_costs.append(movementCost)
					temp_path.append(step)

					# keyerror when position has decimals, like (2,45, 2)
					try:
						int_position = (int(self.position[0]), int(self.position[1]))
						for o in self.fieldd.Env.observe(int_position):
							self.fieldd.costs[o[0]] = o[1]
					# no need to update costs when keyerror, only full (INT, INT) positions have costs
					except KeyError:
						pass

					if self.position == self.s_goal:
						self.path.append(temp_path)
						break

			#print(temp_path)
			temp_path = []
			self.print_env()
			self.fieldd.ChangeEdgeCosts(new_observed_obs)
			new_observed_obs = []
			#time = 0.0
			#yield self.path, self.fieldd.visited, self.totalPathCost
		#for r in self.route:
		#	print(r)
		#print(self.totalPathCost)
		#self.print_env()
		"""print(self.path)
		print(self.pathcosts)
		sum = 0
		for cost in self.pathcosts:
			sum = sum + cost
		print(sum)
		print(self.totalPathCost)
		print(self.fieldd.obs - self.fieldd.border)
		wait = input("return path, visited and costs...")"""
		yield self.path, self.fieldd.visited, self.totalPathCost
		#print(self.fieldd.OPEN.enumerate())
		#print(self.fieldd.g)
		#print(self.fieldd.rhs)
		#print("-------------------------------")
		self.fieldd.visited = []
		#self.fieldd.init(self.path)
		#print(self.fieldd.OPEN.enumerate())
		#print(self.fieldd.g)
		#print(self.fieldd.rhs)
		#wait = input("return...")

	def observe(self):
		int_position = (int(self.position[0]), int(self.position[1]))
		return self.fieldd.Env.get_obstacles(int_position)

	def print_env(self):
		for i in range(-1, self.fieldd.Env.y_range+1):
			for j in range(-1, self.fieldd.Env.x_range+1):
				if (i, j) in self.fieldd.obs:
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
