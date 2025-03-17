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
import pandas as pd

#Scenerio initilization
scenario_number = 7

N, S, Nj, wij, Hij, Vr, Rrsi, hw, Rr = scenario_setting(scenario_number) # insert sceneraio number
# N is number of robots
# S sensor type
# Nj is set of robots for each sensor type
# wij weight for robot i with j type senosr
# Hij sensor j health in i robot
# Rrsi sensor range of robot i with sensor type
# hw is sensor health, used for baseline calcuation only   

# Starting position of the robots
initial_conditions = np.asarray([[1.25, 0.25, 0], [1, 0.5, 0], [1, -0.5, 0],
                                 [-1, 0.75, 0], [0.1, 0.2, 0], [0.2, -0.6, 0],
                                 [-0.75, -0.1, 0], [-1, 0, 0], [0.8, -0.25, 0], [1.3, -0.4, 0]])


robo = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=False, initial_conditions=initial_conditions[0:N].T)

# How many iterations do we want (about N*0.033 seconds)
iterations = 600

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
y_min = -1.5
y_max = 1.5
res = 0.05
#x_global_values = np.arange(x_min,x_max+res,res)
#y_global_values = np.arange(y_min,y_max+res,res) 

# Plot workspace boundary
boundary_points = [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min], [x_min, y_min]]
bound_x, bound_y = zip(*boundary_points)
#square, = robo.axes.plot(bound_x, bound_y, 'black', linewidth=2)
robo.axes.set_xlim(x_min - 0.2, x_max + 0.2)
robo.axes.set_ylim(y_min - 0.2, y_max + 0.3)

# Initial plots
x = robo.get_poses()
x_i = uni_to_si_states(x)
current_x = x_i[0,:,None]
current_y = x_i[1,:,None]
g = robo.axes.scatter(x_i[0, :], x_i[1, :], s=np.pi / 4 * safety_radius_marker_size, marker='o', facecolors='none', edgecolors=CM, linewidth=3)

# Robot number text generator
robot_number_text = np.empty((3, 0))
for jj in range(1, N + 1):
    robot_number_text = np.append(robot_number_text, '{0}'.format(jj))

# Initial labeling of robots
robot_labels = [robo.axes.text(x_i[0, kk], x_i[1, kk] + 0.2, robot_number_text[kk], fontsize=font_size, color='b', fontweight='bold', horizontalalignment='center', verticalalignment='center', zorder=0)
                for kk in range(0, N)]

robo.axes.scatter(x[0,:], x[1,:], s=35, color= [CM[i] for i in range(N)], marker='x') #Initial point mark
#robo.axes.text(-0.9,y_max+0.1, "Proposed Unified Coverage, Scenario = 1", fontsize=12, fontweight='bold') #Plot title
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
convergence_flag = True
iteration_ =[]
terminate_flag = [False for _ in range(N)]

for k in range(iterations):
    print('iteration k = ', k)
    # Get the poses of the robots and convert to single-integrator poses
    x = robo.get_poses()
    x_si = uni_to_si_states(x)
    for r in range(N):
        dist = np.sqrt(np.square(x_si[0,r,None]-current_x[r]) + np.square(x_si[1,r,None]-current_y[r]))
        dist_all = dist_all + dist
    cumulative_distance.append(float(dist_all))
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
    # weigth = 0.1 * np.array([1, 1, 1, 4, 4])
    for n in range(len(S)):
        for m in Nj[n]:
            index = Nj[n].index(m)
            weight[m-1] += Hij[n][index] * Rrsi[n][index]
    weight = weight / np.sum(weight)
    print("weight", weight)
    locations = [[] for _ in range(N)]
    #for s in range(len(S)):
    for ix in np.arange(x_min,x_max+res,res):
        for iy in np.arange(y_min,y_max+res,res):
            importance_value = get_sensor((ix,iy))
            distances = np.zeros(N)
            for robots in range(N):
                if k ==0:
                    distances[robots] = np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))
                else:
                    distances[robots] = np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))

            # print("distances", distances)
            min_index = np.argmin(distances)
            c_v[min_index][0] += ix * importance_value
            c_v[min_index][1] += iy * importance_value
            w_v[min_index] += 1
            locations[min_index].append([ix, iy])
    locations_new = [[] for _ in range(N)]
    for robots in range(N):
        for point in locations[robots]:
            #calculate_distance(point,[current_x,current_y] )


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

    if k==0:
        Vr0 = [1 for r in range(N)]
        weight0 =[1/N for r in range(N)]
        hw0 = [1/N for r in range(N)]
        Rr0 = [1 for r in range(N)]
        Hg, Hp, Ht, Hr, Hgen = cost(N,locations,[current_x,current_x],Vr0,weight0,hw0,Rr0)
    else:
        Hg, Hp, Ht, Hr, Hgen = cost(N,locations,[current_x,current_x],Vr,weight,hw,Rr)

    locational_cost.append(float(Hg[0]))
    health_cost.append(float(Hp[0]))
    mobility_cost.append(float(Ht[0]))
    range_cost.append(float(Hr[0]))
    proposed_cost.append(float(Hgen[0]))
    iteration_.append(k)

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
    print(terminate_flag)
    diff = np.linalg.norm(x_si[:2, :] - prev_x, axis=0)
    each = np.linalg.norm(x_si[:2, :] - prev_x, axis=0).sum()
    print("diff", each)
    #if diff < 0.002:
    if all(d <=1e-3 for d in diff):
        if convergence_flag:
            converged_T = k
            print("Converged")
            title_ = f'Baseline-4, Scenario = {scenario_number}, Converged at t = {converged_T}'
            print(title_)
            robo.axes.text(-1.5,y_max+0.1, title_, fontsize=12, fontweight='bold') #Plot title
            plt.savefig(f'./plot/plot/baseline4_s{scenario_number}.png')
            convergence_flag = False
        #time.sleep(5)
       # break
    # Update the previous positions
    prev_x = x_si[:2, :]

# CSV file saving
# Convert lists into a DataFrame
df = pd.DataFrame({'iteration': iteration_, 'locational_cost': locational_cost, 'health_cost': health_cost, 'mobility_cost': mobility_cost, 'range_cost': range_cost, 'proposed_cost': proposed_cost,'cumulative_distance': cumulative_distance})
# Save to CSV
df.to_csv(f'./plot/cost/baseline4_cost_all{scenario_number}.csv', index=False)  # `index=False` to avoid adding row indices

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
robo.call_at_scripts_end()
