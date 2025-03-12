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
import csv
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
wij = np.ones((len(S),N ))
wij[0,6] =0.5

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

# Saving the list to a CSV file
def save_list_to_csv(my_list, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for item in my_list:
            writer.writerow([item])

    print(f"List saved to {filename}")

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
np.random.seed(50)
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
square, =robo.axes.plot(bound_x, bound_y, 'b-',linewidth = 8) 
robo.axes.set_xlim(x_min-0.2,x_max+0.2) 
robo.axes.set_ylim(y_min-0.2,y_max+0.2)  

# Initial plots
x = robo.get_poses()
x_si = uni_to_si_states(x)
current_x = x_si[0,:,None]
current_y = x_si[1,:,None]
#g = robo.axes.scatter(x[0,:], x[1,:], s=np.pi/4*safety_radius_marker_size, marker='o', facecolors='none',edgecolors=CM,linewidth=3)


# robot number text generator
robot_number_text = np.empty((3,0))
for jj in range(1,N+1):
	robot_number_text = np.append(robot_number_text,'{0}'.format(jj))

# initial labeling of robots
robot_labels = [robo.axes.text(x[0,kk],x[1,kk]+0.2,robot_number_text[kk],fontsize=font_size, color='b',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
for kk in range(0,N)]

robo.step() # Iterate the simulation

# variables to record
pose = []
control_velocity_input =[]
centroid = []
control_input =[]
#dist_robot = [[0] for _ in range(N)]
#cumulative_distance = np.zeros(N)
iteration_list =[]
dist_all =0
dist_all_each_iteration =[]
previous_centroid =[]
terminate_flag = [False for _ in range(N)]
Hg = [] #locational cost

for k in range(iterations):
    print('iteration k = ', k)
    # Get the poses of the robots
    x = robo.get_poses()
    x_si = uni_to_si_states(x)
    for r in range(N):
        dist = math.sqrt(np.square(x_si[0,r,None]-current_x[r]) + np.square(x_si[1,r,None]-current_y[r]))
        dist_all = dist_all + dist
        #cumulative_distance[r] = cumulative_distance[r] + dist
        #dist_robot[r].append(cumulative_distance[r])
    dist_all_each_iteration.append(dist_all)
    iteration_list.append(k)
    
    current_x = x_si[0,:,None]
    current_y = x_si[1,:,None]

    # updating position of robot labels
    for q in range(N):
        robot_labels[q].set_position([x_si[0,q],x_si[1,q]+0.2])
        robot_labels[q].set_fontsize(determine_font_size(robo,font_size_m))

    # Update Plotted Visualization
    #g.set_offsets(x[:2,:].T)
    # This updates the marker sizes if the figure window size is changed. 
    # This should be removed when submitting to the Robotarium.
    #g.set_sizes([determine_marker_size(robo,safety_radius)])

    sum_cord = np.zeros((N, 2))
    num_points = np.zeros(N)
    locations = [[] for _ in range(N)]
    lcost =0
    for i, x_pos in enumerate(x_global_values):
        for j, y_pos in enumerate(y_global_values):
            importance_value = get_sensor(1,(x_pos,y_pos))
            distances =[]
            for r in range(N):
                distances.append(math.sqrt(np.square(x_pos-current_x[r]) + np.square(y_pos-current_y[r]))-wij[0,r])
            min_index = np.argmin(distances)
            lcost = distances[min_index]
            sum_cord[min_index][0] += x_pos*importance_value
            sum_cord[min_index][1] += y_pos*importance_value
            num_points[min_index] += 1
            locations[min_index].append([x_pos, y_pos])

    
    if k>0:
        [plot.remove() for plot in hull_figHandles] 
    hull_figHandles =[]
    for r in range(N):
        q_points = np.array(locations[r])
        hull = ConvexHull(q_points)
        boundary_points = q_points[hull.vertices]
        xss, yss = boundary_points[:, 0], boundary_points[:, 1]
        xss = np.concatenate((xss, [xss[0]]))   # hull.vertices does not provide closed boundary, adding a clyclic vertices for enclosed geometry
        yss = np.concatenate((yss, [yss[0]]))
        hullHandle, =  (robo.axes.plot(xss, yss,'b-',linewidth =8))
        hull_figHandles.append(hullHandle)



    si_velocities = np.zeros((2,N))
    #Cx = []
    #Cy =[]

    for r in range(N):
        centroid_x = 0
        centroid_y = 0
        if not num_points[r]==0:
            centroid_x = sum_cord[r][0]/num_points[r]
            centroid_y = sum_cord[r][1]/num_points[r] 
            centroid_position_difference_x  = centroid_x-current_x[r]    
            centroid_position_difference_y  = centroid_y-current_y[r]
            terminate_flag[r] = (abs(centroid_position_difference_x) < 0.01 and abs(centroid_position_difference_y) <0.01)
            si_velocities[0][r]= 1*(centroid_x-current_x[r])
            si_velocities[1][r] = 1*(centroid_y-current_y[r])
            #Cx.append(centroid_x)
            #Cy.append(centroid_y)

    if all(flag == True for flag in terminate_flag):
        print("Converged")
        time.sleep(5)
        break
    robo.axes.scatter(x[0,:], x[1,:], s=5, color= ["red" for i in range(N)], marker='x')

    # Make sure that the robots don't collide
    #si_velocities = si_barrier_cert(si_velocities, x_si)

    # Transform the single-integrator dynamcis to unicycle dynamics
    dxu = si_to_uni_dyn(si_velocities, x)

    # Set the velocities of the robots
    robo.set_velocities(np.arange(N), dxu)

    #recording variables
    #pose.append(x)
    #centroid.append([Cx,Cy])
    #control_velocity_input.append(si_velocities)
    #control_input.append(dxu)

    # Iterate the simulation
    robo.step()

save_list_to_csv(dist_all_each_iteration, './csv/cumulativeDistanceTravel_baseline0.csv')
#Call at end of script to print debug information and for your script to run on the Robotarium server properly
robo.call_at_scripts_end()