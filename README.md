# pathfinding-simulator
Pathfinding simulator software for my Thesis.

Plotting requires matplotlib

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
