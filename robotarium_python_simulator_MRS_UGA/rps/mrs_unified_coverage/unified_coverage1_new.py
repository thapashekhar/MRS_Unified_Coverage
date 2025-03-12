import rps.robotarium as robotarium
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from scipy.spatial import ConvexHull, Voronoi
import numpy as np
import matplotlib.pyplot as plt

# Instantiate Robotarium object
N = 5


# L = completeGL(N)  # Connectivity of robots (complete graph)

# Sensor type
S = [1, 2]

# Set of Robots for each sensor type Nj
Nj = [[] for _ in range(len(S))]
Nj[0] = [1, 2, 3]  # Set of robots with sensor type 1
Nj[1] = [3, 4, 5]  # Set of robots with sensor type 2
# Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
# Weights of robot i for sensor type j
wij = np.zeros((N, len(S)))

# Health of robots for each sensor type hij (normalized)
Hij = [[] for _ in range(len(S))]
Hij[0] = [1, 0.5, 1]  # Sensor health of type 1
Hij[1] = [1, 0.5, 1]   # Sensor health of type 2
# H = np.ones(N)  # Sensor health of each robot
# Velocity of robots (m/s)
Vr = np.array([1, 1, 1, 1, 1])

# Range of robots Rrsi (normalized)
Rrsi = [[] for _ in range(len(S))]
Rrsi[0] = [1, 0.5, 1]  # Range of robot with sensor type 1
Rrsi[1] = [1, 0.5, 1]  # Range of robot with sensor type 2

# Starting position of the robots
initial_conditions = np.asarray([[1.25, 0.25, 0], [1, 0.5, 0], [1, -0.5, 0],
                                 [-1, 0.75, 0], [0.1, 0.2, 0], [0.2, -0.6, 0],
                                 [-0.75, -0.1, 0], [-1, 0, 0], [0.8, -0.25, 0], [1.3, -0.4, 0]])
initial_conditions = np.asarray([[1.25, 0.25, 0], [1, 0.5, 0], [1, -0.5, 0],
                                 [-1, 0.75, 0], [0.1, 0.2, 0]])
# weight_i = np.zeros(N)
# # weight_i[0] = 1
# weight_i = np.array([1, 2, 1, 1, 4, 1, 1, 1, 1, 1])

robo = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=False, initial_conditions=initial_conditions[0:N].T)

# How many iterations do we want (about N*0.033 seconds)
iterations = 500

# We're working in single-integrator dynamics, and we don't want the robots
# to collide or drive off the testbed.  Thus, we're going to use barrier certificates
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

safety_radius = 0.1
# Generated a connected graph Laplacian (for a cylce graph).
L = completeGL(N)

CM = [
    '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
    '#aec7e8', '#ffbb78', '#98df8a', '#ff9896', '#c5b0d5',
    '#c49c94', '#f7b6d2', '#c7c7c7', '#dbdb8d', '#9edae5'
    ]

safety_radius_marker_size = determine_marker_size(robo, safety_radius)  # Will scale the plotted markers to be the diameter of provided argument (in meters)
font_height_meters = 0.1
font_height_points = determine_font_size(robo, font_height_meters)  # Will scale the plotted font height to that of the provided argument (in meters)
marker_size_goal = determine_marker_size(robo, 0.2)
font_size_m = 0.1
font_size = determine_font_size(robo, font_size_m)
line_width = 5

x_min = -1.5
x_max = 1.5
y_min = -1
y_max = 1
res = 0.05

# Plot workspace boundary
boundary_points = [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min], [x_min, y_min]]
bound_x, bound_y = zip(*boundary_points)
square, = robo.axes.plot(bound_x, bound_y, 'b-', linewidth=5)
robo.axes.set_xlim(x_min - 0.2, x_max + 0.2)
robo.axes.set_ylim(y_min - 0.2, y_max + 0.2)

# Initial plots
x_i = robo.get_poses()
g = robo.axes.scatter(x_i[0, :], x_i[1, :], s=np.pi / 4 * safety_radius_marker_size, marker='o', facecolors='none', edgecolors=CM, linewidth=3)

# Robot number text generator
robot_number_text = np.empty((3, 0))
for jj in range(1, N + 1):
    robot_number_text = np.append(robot_number_text, '{0}'.format(jj))

# Initial labeling of robots
robot_labels = [robo.axes.text(x_i[0, kk], x_i[1, kk] + 0.2, robot_number_text[kk], fontsize=font_size, color='b', fontweight='bold', horizontalalignment='center', verticalalignment='center', zorder=0)
                for kk in range(0, N)]

robo.step()  # Iterate the simulation
x_global_values = np.arange(x_min, x_max + res, res)
y_global_values = np.arange(y_min, y_max + res, res)
hull_local_handler = []

cost_list = []
prev_x = np.zeros((2, N))

for k in range(iterations):
    print('iteration k = ', k)
    # Get the poses of the robots and convert to single-integrator poses
    x = robo.get_poses()
    x_si = uni_to_si_states(x)
    current_x = x_si[0,:,None]
    current_y = x_si[1,:,None]
    
    for q in range(N):
        robot_labels[q].set_position([x_si[0, q], x_si[1, q] + 0.2])
        robot_labels[q].set_fontsize(determine_font_size(robo, font_size_m))

    # Update Plotted Visualization
    g.set_offsets(x[:2, :].T)
    # This updates the marker sizes if the figure window size is changed.
    # This should be removed when submitting to the Robotarium.
    g.set_sizes([determine_marker_size(robo, safety_radius)])

    c_v = np.zeros((N,2))
    w_v = np.zeros(N)
    weight = np.zeros(N)
    locations = [[] for _ in range(N)]
    # weigth = 0.1 * np.array([1, 1, 1, 4, 4])
    for n in range(len(S)):
        for m in Nj[n]:
            index = Nj[n].index(m)
            weight[m-1] = Hij[n][index] * Rrsi[n][index]
    weight = weight / np.sum(weight)
    print("weight", weight)
    cost = 0
    for ix in np.arange(x_min,x_max,res):
        for iy in np.arange(y_min,y_max,res):
            importance_value = 1
            distances = np.zeros(N)
            for robots in range(N):
                distances[robots] = (np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))- weight[robots])/Vr[robots]
                cost += ((np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))- weight[robots])/Vr[robots]**2
            # print("distances", distances)
            
            min_index = np.argmin(distances)
            c_v[min_index][0] += ix * importance_value
            c_v[min_index][1] += iy * importance_value
            w_v[min_index] += 1
            locations[min_index].append([ix, iy])

   
         
       
    if k > 0:
        for plot in hull_figHandles:
            plot.remove()
    hull_figHandles =[]
    for r in range(N):
        q_points = np.array(locations[r])
        hull = ConvexHull(q_points)
        boundary_points = q_points[hull.vertices]
        xss, yss = boundary_points[:, 0], boundary_points[:, 1]
        xss = np.concatenate((xss, [xss[0]]))   # hull.vertices does not provide closed boundary, adding a clyclic vertices for enclosed geometry
        yss = np.concatenate((yss, [yss[0]]))
        # hullHandle, =  (robo.axes.plot(xss, yss,'b-',linewidth =3))
        hullHandle, = (robo.axes.plot(xss, yss, color=CM[r], linewidth=3))
        hull_figHandles.append(hullHandle)  
    # Initialize the single-integrator control inputs
    si_velocities = np.zeros((2, N))
    
  
      
    for robots in range(N):
       c_x = 0
       c_y = 0
       if not w_v[robots] == 0:
          c_x = c_v[robots][0] / w_v[robots]
          c_y = c_v[robots][1] / w_v[robots]  
                    
            
          si_velocities[:, robots] = 1 * [(c_x - current_x[robots][0]), (c_y - current_y[robots][0] )]

    print("cost", cost)
    cost_list.append(cost[0])
    # Use the barrier certificate to avoid collisions
    si_velocities = si_barrier_cert(si_velocities, x_si)

    
    # Transform single integrator to unicycle
    dxu = si_to_uni_dyn(si_velocities, x)
    # print("dxu", dxu)
    for robots in range(N):
        dxu = np.clip(dxu, -Vr[robots], Vr[robots])
    # Set the velocities of agents 1,...,N
    robo.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    robo.step()
    
    # Calculate the change in positions
    diff = np.linalg.norm(x_si[:2, :] - prev_x, axis=0).sum()
    print("diff", diff)
    if diff < 0.005:
        break
        

    # Update the previous positions
    prev_x = x_si[:2, :]
    
plt.figure()
print(cost_list)
plt.plot(cost_list)
plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('Cost vs. Iteration')
plt.grid(True)
plt.show()
#Call at end of script to print debug information and for your script to run on the Robotarium server properly
robo.call_at_scripts_end()
