import matplotlib.pyplot as plt
import numpy as np
from numpy.random import uniform 

#------Initialization-----
#Set the number of particles
N = 20000

#Draw samples from a uniform distribution (initial distribution)
pts = uniform(-1, 1, (N, 2))

#Compute the weights

#Normalize the weights

#-----Main Loop (Iterations)-----
#Use importance distribution

#Update weights

#Normalize weights

#Conduct resampling