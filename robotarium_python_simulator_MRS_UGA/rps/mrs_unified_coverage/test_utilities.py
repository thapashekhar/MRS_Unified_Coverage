import numpy as np
import csv


def scenario_setting(scenario_number):
    if scenario_number == 1:
        N = 5# Instantiate Robotarium object
        S = [1]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3, 4, 5]  # Set of robots with sensor type 1
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))
        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1,1,1,1,1]  # Sensor health of type 1
        hw = [0.2,0.2,0.2,0.2,0.2]
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1
        Rr = [1, 1, 1, 1, 1]
       
    elif scenario_number == 2:
        N = 5# Instantiate Robotarium object
        S = [1, 2]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3]  # Set of robots with sensor type 1
        Nj[1] = [3, 4, 5]  # Set of robots with sensor type 2
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1, 1, 1]  # Sensor health of type 1
        Hij[1] = [1, 1, 1]   # Sensor health of type 2
        hw = [0.16,0.16,0.33,0.16,0.16]
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 1]  # Range of robot with sensor type 2
        Rr = [1, 1, 1, 1, 1] # range of robot 3 is average of both sensor 1 and 2
        
    elif scenario_number == 3:
        N = 5# Instantiate Robotarium object
        S = [1]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3, 4, 5]  # Set of robots with sensor type 1
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1,1,1,1,1]  # Sensor health of type 1
        hw = [0.2,0.2,0.2,0.2,0.2]
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5, 1, 1]  # Range of robot with sensor type 1
        Rr = [1, 1, 0.5, 1, 1]

    elif scenario_number == 4:
        N = 5# Instantiate Robotarium object
        S = [1]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3, 4, 5]  # Set of robots with sensor type 1
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))
        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1,1,1,1,1]  # Sensor health of type 1
        hw = [0.2,0.2,0.2,0.2,0.2]
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1
        Rr = [1, 1, 1, 1, 1]
        

    elif scenario_number == 5:
        N = 5# Instantiate Robotarium object
        S = [1]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3, 4, 5]  # Set of robots with sensor type 1
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))
        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1,1,0.5,1,1]  # Sensor health of type 1
        hw = [0.2,0.2,0.2,0.2,0.2]
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1
        Rr = [1, 1, 1, 1, 1]

    elif scenario_number == 6:
        N = 5# Instantiate Robotarium object
        S = [1, 2]# Sensor type
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
        hw = [0.16,0.16,0.33,0.16,0.16]
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 0.5, 1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 0.5, 1]  # Range of robot with sensor type 2
        Rr = [1, 0.5, 1, 0.5, 1]

    elif scenario_number == 7:
        N = 10 # Instantiate Robotarium object
        S = [1, 2]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3,4,5]  # Set of robots with sensor type 1
        Nj[1] = [6, 7, 8, 9,10]  # Set of robots with sensor type 2
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1, 1, 1,1,1]  # Sensor health of type 1
        Hij[1] = [1, 1, 1,1,1]   # Sensor health of type 2
        hw = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1,2,2,2,2,2])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 1,1,1]  # Range of robot with sensor type 2
        Rr = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]        
    elif scenario_number == 8:
        N = 10 # Instantiate Robotarium object
        S = [1, 2]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3,4,5]  # Set of robots with sensor type 1
        Nj[1] = [6, 7, 8, 9,10]  # Set of robots with sensor type 2
        hw = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1, 1, 1,1,1]  # Sensor health of type 1
        Hij[1] = [1, 1, 1,1,1]   # Sensor health of type 2
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1,1,1,1,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2  
        Rr = [1, 1, 0.5, 1, 1, 1, 1, 0.5, 1, 1]  
    elif scenario_number == 9:
        N = 10 # Instantiate Robotarium object
        S = [1, 2]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3,4,5]  # Set of robots with sensor type 1
        Nj[1] = [6, 7, 8, 9,10]  # Set of robots with sensor type 2
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1, 1, 1,1,1]  # Sensor health of type 1
        Hij[1] = [1, 1, 1,1,1]   # Sensor health of type 2
        hw = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1,1,1,2,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2
        Rr = [1, 1, 0.5, 1, 1, 1, 1, 0.5, 1, 1]    
    elif scenario_number == 10:
        N = 10 # Instantiate Robotarium object
        S = [1, 2]# Sensor type
        # Set of Robots for each sensor type Nj
        Nj = [[] for _ in range(len(S))]
        Nj[0] = [1, 2, 3,4,5]  # Set of robots with sensor type 1
        Nj[1] = [6, 7, 8, 9,10]  # Set of robots with sensor type 2
        # Nj = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]  # Sensor type of each robot
        # Weights of robot i for sensor type j
        wij = np.zeros((N, len(S)))

        # Health of robots for each sensor type hij (normalized)
        Hij = [[] for _ in range(len(S))]
        Hij[0] = [1, 1, 0.5,1,1]  # Sensor health of type 1
        Hij[1] = [1, 1, 0.5,1,1]   # Sensor health of type 2
        hw = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1,1,1,2,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2 
        Rr = [1, 1, 0.5, 1, 1, 1, 1, 0.5, 1, 1] 
        

    return N,S,Nj,wij,Hij,Vr,Rrsi,hw,Rr

# Density function
def get_sensor(q,sensorType =1):
    phi = 1
    return phi

# Saving the list to a CSV file
def save_list_to_csv(my_list, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for item in my_list:
            writer.writerow([item])

    print(f"List saved to {filename}")

def calculate_distance(point1, point2):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def cost(N,locations,robot_position, velocity, weightr,hw,Rr):
    locational_cost = 0
    health_cost =0
    mobility_cost =0
    range_cost = 0
    proposed_cost = 0
    for r in range(N):
        prop_cost = [(0.5*(calculate_distance([robot_position[0][r],robot_position[1][r]], point)-weightr[r])/velocity[r]**2)*get_sensor(point[0],point[1]) for point in locations[r]] #weight[r] is unified weight (hsi*Rrsi) for robot r 
        locat_ = [calculate_distance([robot_position[0][r],robot_position[1][r]], point)*get_sensor(point[0],point[1]) for point in locations[r]]
        health = [0.5*(calculate_distance([robot_position[0][r],robot_position[1][r]], point)-hw[r])*get_sensor(point[0],point[1]) for point in locations[r]]  #hwr is sensor health of robot r
        temporal = [(1/velocity[r]**2)*(calculate_distance([robot_position[0][r],robot_position[1][r]], point))*get_sensor(point[0],point[1]) for point in locations[r]]
        #RangE = [calculate_distance([robot_position[0][r],robot_position[1][r]], point)*get_sensor(point[0],point[1]) for point in locations[r]] # needs correction, will be corrected in next version
        RangeValue = []
        for i, point in enumerate(locations[r]):
            dist_ = calculate_distance([robot_position[0][r],robot_position[1][r]], point)
            if dist_ <= Rr[r]:
                RangeValue.append(dist_*get_sensor(point[0],point[1]))
            else:
                RangeValue.append((Rr[r]**2)*get_sensor(point[0],point[1])) 
        locational_cost+=sum(locat_)
        health_cost+=sum(health)
        mobility_cost+=sum(temporal)
        range_cost+=sum(RangeValue) # needs improvement
        proposed_cost += sum(prop_cost)

    return locational_cost, health_cost, mobility_cost, range_cost, proposed_cost
    
