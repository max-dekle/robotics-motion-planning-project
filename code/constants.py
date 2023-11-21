import numpy as np

# Robot dimensions and physical constants
R = 0.0475
L = 0.235
W = 0.15

# Transformation matrices
T_B0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
M_OE = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
B_LIST = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T

# Chassis configuration matrix F
F = R / 4 * np.array([[-1/(L+W), 1/(L+W), 1/(L+W), -1/(L+W)], [1, 1, 1, 1], [-1, 1, -1, 1]])

# Full body configuration matrix F_6
F_6 = R / 4 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [-1/(L+W), 1/(L+W), 1/(L+W), -1/(L+W)], [1, 1, 1, 1], [-1, 1, -1, 1], [0, 0, 0, 0]])

# Initial end-effector configuration matrices
TSE_INI = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
TCE_GRP = np.array([[-np.sqrt(2)/2, 0, np.sqrt(2)/2, 0], [0, 1, 0, 0], [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0], [0, 0, 0, 1]])
TCE_STA = np.array([[-np.sqrt(2)/2, 0, np.sqrt(2)/2, 0], [0, 1, 0, 0], [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0.15], [0, 0, 0, 1]])

# Time and speed constants
TIMESTEP = 0.01
MAX_SPEED = 15
