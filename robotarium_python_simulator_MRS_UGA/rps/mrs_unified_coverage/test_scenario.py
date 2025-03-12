import numpy as np


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
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1
       
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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 1]  # Range of robot with sensor type 2
        
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
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5, 1, 1]  # Range of robot with sensor type 1

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
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1

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
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1, 1, 1]  # Range of robot with sensor type 1

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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 0.5, 1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 0.5, 1]  # Range of robot with sensor type 2

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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1,2,2,2,2,2])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 1,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 1,1,1]  # Range of robot with sensor type 2        
    elif scenario_number == 8:
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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 1, 1, 1,1,1,1,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2  
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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1,1,1,2,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2  
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
        # H = np.ones(N)  # Sensor health of each robot
        # Velocity of robots (m/s)
        Vr = np.array([1, 1, 2, 1, 1,1,1,2,1,1])

        # Range of robots Rrsi (normalized)
        Rrsi = [[] for _ in range(len(S))]
        Rrsi[0] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 1
        Rrsi[1] = [1, 1, 0.5,1,1]  # Range of robot with sensor type 2 

    return N,S,Nj,wij,Hij,Vr,Rrsi
    
