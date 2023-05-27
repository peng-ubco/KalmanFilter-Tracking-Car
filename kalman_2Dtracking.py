import numpy as np
from simulator.sim_2d import simulator_kf

# Simulator options.
sim_opt = {}
sim_opt['FIG_SIZE'] = [8, 8]

sim_opt['DRIVE_CIRCLE'] = False
# If False, measurements will be position_x, position_y.
# If True, measurements will be position_x, position_y, and current angle of the car.
sim_opt['MEASURE_ANGLE'] = False  # Required if driving in circle.
sim_opt['CONTROL_INPUTS'] = False  # Required if driving in circle.
sim_opt['METHOD'] = 'method_1'

class KF_Circular:
    def __init__(self):
        self.pre_t = 0
        if sim_opt['DRIVE_CIRCLE'] == False:
            # Initial State Vector [[pos_x], [pos_y], [velocity_x], [velocity_y]]
            self.x = np.array([[0.],
                               [0.],
                               [0.],
                               [0.]])

            # Initial Estimate Uncertainty
            self.P = np.array([[1000, 0, 0, 0],
                               [0, 1000, 0, 0],
                               [0, 0, 1000, 0],
                               [0, 0, 0, 1000]])

            # State Transition Matrix
            self.F = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

            # Observation Matrix
            self.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]])

            # Initial Measurement Uncertainty
            self.R = np.array([[(np.random.rand(1)[0] - 0.5) * 0.5, 0.],
                               [0., (np.random.rand(1)[0] - 0.5) * 0.5]])

            self.I = np.eye(4, )

            # Process noise covariance Matrix
            self.Q = np.array([[0.1, 0., 0., 0.],
                               [0., 0.1, 0., 0.],
                               [0., 0., 0.1, 0.],
                               [0., 0., 0., 0.1]])

        else:
            # Initial State Vector [[pos_x], [pos_y], [theta], [velocity_x], [velocity_y], [theta_dot]]
            self.x = np.array([[0.],
                               [0.],
                               [0.],
                               [0.],
                               [0.],
                               [0.]])

            # Initial Estimate Uncertainty
            self.P = np.array([[1000, 0, 0, 0, 0, 0],
                               [0, 1000, 0, 0, 0, 0],
                               [0, 0, 1000, 0, 0, 0],
                               [0, 0, 0, 1000, 0, 0],
                               [0, 0, 0, 0, 1000, 0],
                               [0, 0, 0, 0, 0, 1000]])

            # State Transition Matrix
            self.F = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 1]])

            # Observation Matrix
            self.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0]])

            # Initial Measurement Uncertainty
            self.R = np.array([[(np.random.rand(1)[0] - 0.5) * 0.5, 0., 0.],
                               [0., (np.random.rand(1)[0] - 0.5) * 0.5, 0.],
                               [0., 0., (np.random.rand(1)[0] - 0.5) * 0.5]])

            self.I = np.eye(6, )

            # Process noise covariance Matrix
            self.Q = np.array([[0.1, 0., 0., 0., 0., 0.],
                               [0., 0.1, 0., 0., 0., 0.],
                               [0., 0., 0.1, 0., 0., 0.],
                               [0., 0., 0., 0.1, 0., 0.],
                               [0., 0., 0., 0., 0.1, 0.],
                               [0., 0., 0., 0., 0., 0.1]])

    def predict(self, t):
        # Calculate dt
        dt = t - self.pre_t
        # Add dt in transition matrix
        if sim_opt['DRIVE_CIRCLE'] == False:
            self.F[0][2] = dt
            self.F[1][3] = dt

        else:
            self.F[0][3] = dt
            self.F[1][4] = dt
            self.F[2][5] = dt

        # Predict
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        return

    def update(self, measurements, t):
        z = np.array([measurements])
        y = z.T - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        # Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # State update
        self.x = self.x + np.dot(K, y)
        self.P = (self.I - np.dot(np.dot(K, self.H), self.P))

        self.pre_t = t


        return [self.x[0], self.x[1]]

    # Required if driving in circle.
    def control_inputs(self, u_steer, u_pedal):
        return

class KF_Noncircle:
    def __init__(self):
        self.pre_t = 0
        # Initial State Vector [[pos_x], [pos_y], [velocity_x], [velocity_y]]
        self.x = np.array([[0.],
                           [0.],
                           [0.],
                           [0.]])

        # Control Input Vector
        self.u = np.array([[0.],
                           [0.],
                           [0.],
                           [0.]])

        # Uncertainty Matrix
        self.P = np.array([[1000., 0., 0, 0],
                           [0., 1000., 0., 0.],
                           [0., 0., 1000., 0.],
                           [0., 0., 0., 1000.]])

        # State Transition Matrix
        self.F = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])

        # Observation Matrix
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])

        # Measurement Uncertainty
        self.R = np.array([[5.0, 0.],
                           [0., 5.0]])

        # Identity Matrix
        self.I = np.eye(4,)

    def predict(self, t):
        # Calculate dt.
        # Put dt into the state transition matrix.
        dt = t - self.pre_t
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ np.transpose(self.F)
        self.P[0, 0] += 0.1
        self.P[1, 1] += 0.1
        self.P[2, 2] += 0.1
        self.P[3, 3] += 0.1
        return

    def update(self, measurements, t):
        Z = np.array([measurements])
        y = np.transpose(Z) - (self.H @ self.x)
        S = self.H @ self.P @ np.transpose(self.H) + self.R
        K = self.P @ np.transpose(self.H) @ np.linalg.inv(S)

        self.x = self.x + (K @ y)
        self.P = (self.I - (K @ self.H)) @ self.P
        self.pre_t = t
        return [self.x[0], self.x[1]]

    # Required if driving in circle.
    def control_inputs(self, u_steer, u_pedal):
        return


if sim_opt['DRIVE_CIRCLE']:
    simulator_kf(sim_opt, KF_Circular)
else:
    simulator_kf(sim_opt, KF_Noncircle)
