"""
Plotting - use matplotlib to plot route and environment/visited nodes
"""

import matplotlib.pyplot as plt
import utils.env
import math

class Plotting:
	def __init__(self, xI, xG, Env):
		self.xI, self.xG = xI, xG
		self.env = Env
		self.obs = self.env.obs

		self.costs = self.env.costs

		self.wait_time = 0.2
		self.ln = 0

		self.values = True

	def update_obs(self, obs):
		self.obs = obs

	def plot_grid(self, name):
		#self.obs = list(self.obs)
		#self.obs.sort()

		obs_y = [x[0] for x in self.obs]
		obs_x = [x[1] for x in self.obs]
		#print(self.obs)

		plt.plot(self.xI[1], self.xI[0], "bs")
		plt.plot(self.xG[1], self.xG[0], "gs")
		plt.plot(obs_x, obs_y, "sk")
		plt.gca().invert_yaxis()
		plt.title(name)
		plt.axis("equal")

	def plot_path(self, path, cl='r', flag=False):
		path_y = [path[i][0] for i in range(len(path))]
		path_x = [path[i][1] for i in range(len(path))]

		if not flag:
			plt.plot(path_x, path_y, linewidth='3', color='r')
		else:
			plt.plot(path_x, path_y, linewidth='3', color=cl)

		plt.plot(self.xI[1], self.xI[0], "bs")
		plt.plot(self.xG[1], self.xG[0], "gs")

		try:
			self.ln.pop(0).remove()
		except AttributeError:
			pass

		self.ln = plt.plot(path_x[-1], path_y[-1], "rs")

		plt.pause(self.wait_time)

	# takes values ranging from 1 to mult and scales to 1 to 0
	def scaleColor(self, val, m):
		s = min((val - 1.0)/(m - 0.99999), 1.0)
		return 1.0 - max(0.0, s)

	def plot_values(self):
		v = self.costs.values()
		k = self.costs.keys()
		m = self.maxCost(v)
		for i in k:
			col = self.scaleColor(self.costs[i], m)
			plt.plot(i[1], i[0], color=(col, col, col), marker='o')

	def plot_visited(self, visited, cl='gray'):
		if self.xI in visited:
			visited.remove(self.xI)

		if self.xG in visited:
			visited.remove(self.xG)

		count = 0

		for x in visited:
			count += 1
			plt.plot(x[1], x[0], color=cl, marker='o')

			if count < len(visited) / 3:
				length = 20
			elif count < len(visited) * 2 / 3:
				length = 30
			else:
				length = 40
			#
			# length = 15

			if count % length == 0:
				plt.pause(0.001)
		plt.pause(0.01)

	def maxCost(self, array):
		max_cost = 0
		for i in array:
			if i > max_cost and i != math.inf:
				max_cost = i
		return max_cost

	def animation(self, path, visited, name):
		self.plot_grid(name)
		plt.pause(self.wait_time)
		cl = self.color_list_2()
		path_combine = []

		# plot values
		if self.values:
			self.plot_values()
		for k in range(len(path)):
			# plot visited nodes
			if not self.values:
				self.plot_visited(visited[k], cl[k])
				#plt.pause(0.2)
			self.plot_path(path[k])
			path_combine += path[k]
			plt.pause(self.wait_time)
		if self.xI in path_combine:
			path_combine.remove(self.xI)
		self.plot_path(path_combine)
		plt.show()

	@staticmethod
	def color_list():
		cl_v = ['silver',
				'wheat',
				'lightskyblue',
				'royalblue',
				'slategray']
		cl_p = ['gray',
				'orange',
				'deepskyblue',
				'red',
				'm']
		return cl_v, cl_p

	@staticmethod
	def color_list_2():
		cl = ['silver',
			  'steelblue',
			  'dimgray',
			  'cornflowerblue',
			  'dodgerblue',
			  'royalblue',
			  'plum',
			  'mediumslateblue',
			  'mediumpurple',
			  'blueviolet',
			  'silver',
			  'steelblue',
			  'dimgray',
			  'cornflowerblue',
			  'dodgerblue',
			  'royalblue',
			  'plum',
			  'mediumslateblue',
			  'mediumpurple',
			  'blueviolet',
			  ]
		return cl
