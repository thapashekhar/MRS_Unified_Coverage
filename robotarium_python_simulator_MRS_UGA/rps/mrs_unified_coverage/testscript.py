#Import Robotarium Utilities
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.graph import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import matplotlib.pyplot as plt
import math
import copy
import time
from scipy.spatial import ConvexHull


#Total number of robots
N = 10

# Laplacian for cycle
L = completeGL(N)  #connectivity of robots (complete graph)

# sensor type
S = [1,2] 

# Set of Robots for each sensor typpe Nj
Nj = [[] for _ in range(len(S))] 
Nj[0] = [1,2,3,4,5]  # set of robots with sensor type 1
Nj[1] = [6,7,8,9,10]  # set of robots with sensor type 2

# weights of robot i for sensor type j
wij = np.ones((N, len(S)))

# Health of robots for each senosr type hij (normalized)
Hij = [[] for _ in range(len(S))] 
Hij[0] = [1,1,1,1,1]  # sensor health of type 1
Hij[1] = [1,1,1,1,1]  # senosr health of type 2

# Velocity of robots (m/s)
Vr = np.ones(N)

# Range of robots Rrsi (normalizied)
Rrsi = [[] for _ in range(len(S))] 
Rrsi[0] = [1,1,1,1,1]  # range of robot with sensor type 1
Rrsi[1] = [1,1,1,1,1] # range of robot with sensor type 2

# Density function
def get_sensor(j,q):
    phi = 1
    return phi

# Maximum iterations
iterations = 1000 #number of steps/ iteration to run the simulation

# Environment size and discretization 
x_min =-1.5
x_max = 1.5
y_min = -1.5
y_max = 1.5
res = 0.05
x_global_values = np.arange(x_min,x_max+res,res)
y_global_values = np.arange(y_min,y_max+res,res) 

# Starting position of the robots
initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],
                                 [-1, 0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],
                                 [-0.75, -0.1, 0],[-1, 0, 0],[0.8, -0.25, 0],[1.3, -0.4, 0]]) 


# Instantiate the Robotarium
robo = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=False, initial_conditions= initial_conditions[0:N].T)
#si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()


# Plotting Parameters
CM = np.random.rand(N,3) # Random Colors
safety_radius = 0.1
safety_radius_marker_size = determine_marker_size(robo,safety_radius) # Will scale the plotted markers to be the diameter of provided argument (in meters)
font_height_meters = 0.1
font_height_points = determine_font_size(robo,font_height_meters) # Will scale the plotted font height to that of the provided argument (in meters)
font_size_m = 0.1
font_size = determine_font_size(robo,font_size_m)
line_width = 5

# Plot workspace boundary
boundary_points = [[x_min,y_min],[x_min,y_max],[x_max,y_max],[x_max,y_min],[x_min,y_min]] 
bound_x, bound_y = zip(*boundary_points) 
square, =robo.axes.plot(bound_x, bound_y, 'b-',linewidth = 5) 
robo.axes.set_xlim(x_min-0.2,x_max+0.2) 
robo.axes.set_ylim(y_min-0.2,y_max+0.2)  

# Initial plots
x = robo.get_poses()
g = robo.axes.scatter(x[0,:], x[1,:], s=np.pi/4*safety_radius_marker_size, marker='o', facecolors='none',edgecolors=CM,linewidth=3)

# robot number text generator
robot_number_text = np.empty((3,0))
for jj in range(1,N+1):
	robot_number_text = np.append(robot_number_text,'{0}'.format(jj))

# initial labeling of robots
robot_labels = [robo.axes.text(x[0,kk],x[1,kk]+0.2,robot_number_text[kk],fontsize=font_size, color='b',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
for kk in range(0,N)]

plt.pause(5)