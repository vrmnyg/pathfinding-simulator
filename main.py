"""
Main classes:

Main
Program execution starts here.

Agent
Selects a Navigator class and uses it
to calculate path to goal. Agent executes until it finds no
further path improvement.
It then collects some statistical data about execution.

Navigator
Navigators job is to calculate path to goal using
the corresponding algorithm. When obstacles are found
navigator recalculates the path to goal from current
position. This is done every time agent finds a blocking
obstacle until goal is found.

Env
Information about the environment (obstacles, cell difficulties)
are found in Env class.
"""

import utils.plotting as plotting
import utils.env as env
from agent import Agent

# this is the main
# program logic starts here
def main():
	#Env = env.Env(5+1, 7+1, True)  # class Env
	#s_start = (0, 2)
	#s_goal = (0, 5)

	# the map of the simulation
	Env = env.Env(7+1, 49+1, True)  # class Env
	s_start = (0, 2)
	#s_start = (2, 3)
	s_goal = (1, 45)

	#Env.print_env(s_start, s_goal)

	#wait = input("PRESS ENTER TO CONTINUE...")

	# TODO make a test for field to check out if no possible solution exist!
	# work in progress
	algo1 = ("FieldD", "Field D* (FD*)")
	algo2 = ("LRTA", "LRTA*")
	algo3 = ("ARA", "ARA*")
	algo4 = ("AD", "AD*")

	# pick the algorith to use here!
	algo = algo1
	#algo = algo2
	#algo = algo3
	#algo = algo4

	# agent receives the map, start point, goal point, and the algorith it uses to navigate
	agent = Agent(Env, s_start, s_goal, algo[0])
	#plot = plotting.Plotting(s_start, s_goal, Env)
	#plot2 = plotting.Plotting(s_start, s_goal, Env)

	#results of the simulation
	path, visited, cost = agent.run()

	# print some statistics
	print("iterations: " + str(len(path)))
	print("visited len: " + str(len(visited)))
	#print(visited)
	print("cost: " + str(cost))
	# plot the Environment and paths
	for i in range(len(path)):
		plot = plotting.Plotting(s_start, s_goal, Env)
		plot.animation(path[i], visited[i], algo[1])

	#plot.animation(path[i], visited[i], algo1[1])
	#plot2.animation(path[0], visited[0], algo1[1])

if __name__ == '__main__':
	main()
