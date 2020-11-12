"""
QueuePrior
This is a priority queue implementation
https://en.wikipedia.org/wiki/Priority_queue
"""

import collections
import heapq
import math

class QueuePrior:

	def __init__(self):
		self.queue = []

	def heapify(self):
		heapq.heapify(self.queue)

	def index(self, s):
		i = -1
		for i in range (0, len(self.queue)):
			if self.queue[i][1] == s:
				return i
		return i

	def exists(self, s):
		if len(self.queue) > 0:
			for i in range (0, len(self.queue)):
				if self.queue[i][1] == s:
					return True
		return False

	def empty(self):
		return len(self.queue) == 0

	def length(self):
		return len(self.queue)

	def update(self, item, priority):
		index = self.index(item)
		if index != -1:
			self.queue[index] = (priority, item)
			heapq.heapify(self.queue)
		else:
			#heapq.heappush(self.queue, (priority, item))  # reorder s using priority
			print("ERROR!")

	def remove(self, item):
		index = self.index(item)
		if index != -1:
			self.queue[index] = self.queue[-1]
			self.queue.pop()
			heapq.heapify(self.queue)
		else:
			print("ERROR!")

	def put(self, item, priority):
		heapq.heappush(self.queue, (priority, item))  # reorder s using priority

	def get(self):
		return heapq.heappop(self.queue)[1]  # pop out the smallest item

	def enumerate(self):
		return self.queue

	def peek(self):
		try:
			return self.queue[0]
		except IndexError:
			return ((math.inf, math.inf), (math.inf, math.inf))
			#return None
