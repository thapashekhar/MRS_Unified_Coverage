import numpy as np


def partition(N):
    x_min = -1.5
    x_max = 1.5
    y_min = -1
    y_max = 1
    res = 0.05
    c_v = np.zeros((N,2))
    w_v = np.zeros(N)
    locations = [[] for _ in range(N)]

    for ix in np.arange(x_min,x_max+res,res):
        for iy in np.arange(y_min,y_max+res,res):
            importance_value = 1
            distances = np.zeros(N)
            for robots in range(N):
                distances[robots] = (np.sqrt(np.square(ix - current_x[robots]) + np.square(iy - current_y[robots]))- weight[robots])/Vr[robots]
            # print("distances", distances)
            min_index = np.argmin(distances)
            c_v[min_index][0] += ix * importance_value
            c_v[min_index][1] += iy * importance_value
            w_v[min_index] += 1
            locations[min_index].append([ix, iy])
    for robots in range(N):
        c_x = 0
        c_y = 0
        if not w_v[robots] == 0:
            c_x = c_v[robots][0] / w_v[robots]
            c_y = c_v[robots][1] / w_v[robots] 
                

 
                        