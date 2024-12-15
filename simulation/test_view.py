import matplotlib.pyplot as plt
#import simulation files
from environment import *
from robot_sim import *
# import pandas as pd
import matplotlib.patches as patches
import matplotlib.animation as animation
import csv
import math


##### INPUT #####

# positions are defined as (i,j) where ((0,0) is in upper left corner, and i,j are the indices of the grid cells, 
# i along the x axis (short side of the grid), j along the y axis
obstacles_pos =  [(0,2),(1,3)]
starting_pos = (0,0,-30,0)
target_pos = (2,5)

task = "RoboNav" # determine grid

env = Environment(task = task, starting_pos = starting_pos, target_pos = target_pos, obstacles = obstacles_pos)
robot_state = State(8.5,8.5,-10,0)
blobs, img = env.take_snapshot(robot_state,max_range = 50)
img.draw_view(robot_state)