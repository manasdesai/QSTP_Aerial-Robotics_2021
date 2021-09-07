This assignment contains the python file for implementing the RRT_algorithm in 3D without obstacles and with obstacles too
The number of iterations play a very key role here as otherwise it is not able to plan a path under 1000 iterations...The algorithm just ends up on a random node in such a case
Also the distance of the random node generated from the nearby nodes also needs to be set appropriately as when I tried with a step length of 10 initially; the number of iterations surpassed 2000 iterations using up all the RAM and eventually the execution crashed.
Afterwards after trying several permutations I was successfully able to plan a path with max_iterations=2000 and step_length of 20 while generating a random node
