import rps.robotarium as robotarium
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np

# Instantiate Robotarium object
N = 5


L = completeGL(N)  # Connectivity of robots (complete graph)
iterations = 1000  # Number of steps/iteration to run the simulation

# Sensor type
S = [1, 2]

# Set of Robots for each sensor type Nj
Nj = [[] for _ in range(len(S))]
Nj[0] = [1, 2, 3, 4, 5]  # Set of robots with sensor type 1
Nj[1] = [6, 7, 8, 9, 10]  # Set of robots with sensor type 2
N_count = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
# Weights of robot i for sensor type j
wij = np.zeros((N, len(S)))

# Health of robots for each sensor type hij (normalized)
Hij = [[] for _ in range(len(S))]
Hij[0] = [1, 1, 1, 1, 1]  # Sensor health of type 1
Hij[1] = [1, 1, 1, 1, 1]  # Sensor health of type 2
H = np.ones(N)  # Sensor health of each robot
# Velocity of robots (m/s)
Vr = np.ones(N) * 2

# Range of robots Rrsi (normalized)
Rrsi = [[] for _ in range(len(S))]
Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1
Rrsi[1] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 2

# Starting position of the robots
initial_conditions = np.asarray([[1.25, 0.25, 0], [1, 0.5, 0], [1, -0.5, 0],
                                 [-1, 0.75, 0], [0.1, 0.2, 0], [0.2, -0.6, 0],
                                 [-0.75, -0.1, 0], [-1, 0, 0], [0.8, -0.25, 0], [1.3, -0.4, 0]])

weight_i = np.zeros(N)
# weight_i[0] = 1
weight_i = np.array([1, 2, 1, 1, 4, 1, 1, 1, 1, 1])

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)

# How many iterations do we want (about N*0.033 seconds)
iterations = 500

# We're working in single-integrator dynamics, and we don't want the robots
# to collide or drive off the testbed.  Thus, we're going to use barrier certificates
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

# Generated a connected graph Laplacian (for a cylce graph).
L = lineGL(N)

x_min = -1.5
x_max = 1.5
y_min = -1
y_max = 1
res = 0.05

 

for k in range(iterations):
    print('iteration k = ', k)
    # Get the poses of the robots and convert to single-integrator poses
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    current_x = x_si[0,:,None]
    current_y = x_si[1,:,None]
  
    c_v = np.zeros((N,2))
    w_v = np.zeros(N)
    weigth = np.zeros(N)
    weigth = np.array([1, 2, 1, 1, 4])
    for ix in np.arange(x_min,x_max,res):
        for iy in np.arange(y_min,y_max,res):
            importance_value = 1
            distances = np.zeros(N)
            for robots in range(N):
                distances[robots] = np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots])- weigth[robots])
            min_index = np.argmin(distances)
            c_v[min_index][0] += ix * importance_value
            c_v[min_index][1] += iy * importance_value
            w_v[min_index] += 1
   
         
                
                
    # Initialize the single-integrator control inputs
    si_velocities = np.zeros((2, N))
    
  
      
    for robots in range(N):
       c_x = 0
       c_y = 0
       if not w_v[robots] == 0:
          c_x = c_v[robots][0] / w_v[robots]
          c_y = c_v[robots][1] / w_v[robots]  
                    
            
          si_velocities[:, robots] = 1 * [(c_x - current_x[robots][0]), (c_y - current_y[robots][0] )]

    # Use the barrier certificate to avoid collisions
    si_velocities = si_barrier_cert(si_velocities, x_si)

    # Transform single integrator to unicycle
    dxu = si_to_uni_dyn(si_velocities, x)

    # Set the velocities of agents 1,...,N
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
