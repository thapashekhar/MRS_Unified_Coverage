import rps.robotarium as robotarium
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from scipy.spatial import ConvexHull, Voronoi
import numpy as np
from test_utilities import *
import time


#Scenerio initilization
N, S, Nj, wij, Hij, Vr, Rrsi, hw = scenario_setting(scenario_number=7) # insert sceneraio number
# N is number of robots
# S sensor type
# Nj is set of robots for each sensor type
# wij weight for robot i with j type senosr
# Hij sensor j health in i robot
# Rrsi sensor range of robot i with sensor type   

# Starting position of the robots
initial_conditions = np.asarray([[1.25, 0.25, 0], [1, 0.5, 0], [1, -0.5, 0],
                                 [-1, 0.75, 0], [0.1, 0.2, 0], [0.2, -0.6, 0],
                                 [-0.75, -0.1, 0], [-1, 0, 0], [0.8, -0.25, 0], [1.3, -0.4, 0]])


robo = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=False, initial_conditions=initial_conditions[0:N].T)

# How many iterations do we want (about N*0.033 seconds)
iterations = 1000

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
res = 0.03
x_global_values = np.arange(x_min,x_max+res,res)
y_global_values = np.arange(y_min,y_max+res,res) 

# Plot workspace boundary
boundary_points = [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min], [x_min, y_min]]
bound_x, bound_y = zip(*boundary_points)
#square, = robo.axes.plot(bound_x, bound_y, 'black', linewidth=3)
robo.axes.set_xlim(x_min - 0.2, x_max + 0.2)
robo.axes.set_ylim(y_min - 0.2, y_max + 0.2)

# Initial plots
x = robo.get_poses()
x_i = uni_to_si_states(x)
g = robo.axes.scatter(x_i[0, :], x_i[1, :], s=np.pi / 4 * safety_radius_marker_size, marker='o', facecolors='none', edgecolors=CM, linewidth=3)

# Robot number text generator
robot_number_text = np.empty((3, 0))
for jj in range(1, N + 1):
    robot_number_text = np.append(robot_number_text, '{0}'.format(jj))

# Initial labeling of robots
robot_labels = [robo.axes.text(x_i[0, kk], x_i[1, kk] + 0.2, robot_number_text[kk], fontsize=font_size, color='b', fontweight='bold', horizontalalignment='center', verticalalignment='center', zorder=0)
                for kk in range(0, N)]

robo.axes.scatter(x[0,:], x[1,:], s=35, color= [CM[i] for i in range(N)], marker='x') #Initial point mark

robo.step()  # Iterate the simulation


# initilization of variables for data recording
prev_x = np.zeros((2, N))
dist_all =0
cumulative_distance =[]
locational_cost =[]
health_cost = []
mobility_cost = []
range_cost = []
proposed_cost =[]

for k in range(iterations):
    print('iteration k = ', k)
    # Get the poses of the robots and convert to single-integrator poses
    x = robo.get_poses()
    x_si = uni_to_si_states(x)
    current_x = x_si[0,:,None]
    current_y = x_si[1,:,None]
    for r in range(N):
        dist = np.sqrt(np.square(x_si[0,r,None]-current_x[r]) + np.square(x_si[1,r,None]-current_y[r]))
        dist_all = dist_all + dist
    cumulative_distance.append(dist_all)
    
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
    # weigth = 0.1 * np.array([1, 1, 1, 4, 4])
    for n in range(len(S)):
        for m in Nj[n]:
            index = Nj[n].index(m)
            weight[m-1] = Hij[n][index] * Rrsi[n][index]
    weight = weight / np.sum(weight)
    print("weight", weight)
    locations = [[] for _ in range(N)]
    #for s in range(len(S)):
    for ix in np.arange(x_min,x_max+res,res):
        for iy in np.arange(y_min,y_max+res,res):
            importance_value = get_sensor((ix,iy))
            distances = np.zeros(N)
            for robots in range(N):
                distances[robots] = (np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))-weight[robots])/Vr[robots] #proposed
                #distances[robots] = np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots])) # Baseline0
                #distances[robots] = np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))/Vr[robots] # Baseline3

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
        xss = np.concatenate((xss, [xss[0]]))   # hull.vertices does not provide closed boundary, adding a cyclic vertices for enclosed geometry
        yss = np.concatenate((yss, [yss[0]]))
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
          # control inputs          
          si_velocities[:, robots] = 1 * [(c_x - current_x[robots][0]), (c_y - current_y[robots][0] )]

    Hg, Hp, Ht, Hr, Hgen = cost(N,locations,[current_x,current_x],Vr,weight,hw) 
    locational_cost.append(Hg)
    health_cost.append(Hp)
    mobility_cost.append(Ht)
    range_cost.append(Hr)
    proposed_cost.append(Hgen)

    # Use the barrier certificate to avoid collisions
    #si_velocities = si_barrier_cert(si_velocities, x_si)

    
    # Transform single integrator to unicycle
    dxu = si_to_uni_dyn(si_velocities, x)
    # print("dxu", dxu)
    for robots in range(N):
        dxu = np.clip(dxu, -Vr[robots], Vr[robots])
    # Set the velocities of agents 1,...,N
    robo.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    robo.step()
    
    robo.axes.scatter(x[0,:], x[1,:], s=5, color= ["red" for i in range(N)], marker='x') # plotting trajectory
    # Calculate the change in positions
    diff = np.linalg.norm(x_si[:2, :] - prev_x, axis=0).sum()
    print("diff", diff)
    if diff < 0.01:
        plt.savefig('./plot/coverageS7.png')
        print("Converged")
        time.sleep(5)
        break

    # Update the previous positions
    prev_x = x_si[:2, :]


save_list_to_csv(locational_cost, './csv/s7/locationalCost.csv')
save_list_to_csv(health_cost, './csv/s7/healthCost.csv')
save_list_to_csv(mobility_cost, './csv/s7/mobilityCost.csv')
save_list_to_csv(range_cost, './csv/s7/rangeCost.csv')
save_list_to_csv(proposed_cost, './csv/s7/proposedCost.csv')
save_list_to_csv(cumulative_distance, './csv/s7/cumulativeDistanceTravel.csv')
#Call at end of script to print debug information and for your script to run on the Robotarium server properly
robo.call_at_scripts_end()
