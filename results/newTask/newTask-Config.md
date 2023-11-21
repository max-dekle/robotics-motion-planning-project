'''newTask'''
# Feedforward + PI Controller
kp = 25
ki = 0.15
Tsc_ini = np.array([[0, -1, 0, 1],
                    [1, 0, 0, 1.0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
Tsc_fin = np.array([[2, 0, 0, 2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])