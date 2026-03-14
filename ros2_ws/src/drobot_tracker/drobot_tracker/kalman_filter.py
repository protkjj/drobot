import numpy as np


class KalmanFilter6D:
    """6D constant-velocity Kalman filter: state = [x, y, z, vx, vy, vz]."""

    def __init__(self, initial_pos, process_noise_std=0.5, measurement_noise_std=0.3):
        # State: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        self.x[:3] = initial_pos

        # State covariance
        self.P = np.eye(6)
        self.P[3:, 3:] *= 10.0  # high uncertainty on initial velocity

        # Measurement matrix: we only observe [x, y, z]
        self.H = np.zeros((3, 6))
        self.H[:3, :3] = np.eye(3)

        # Measurement noise
        self.R = np.eye(3) * measurement_noise_std ** 2

        self.process_noise_std = process_noise_std

    def predict(self, dt):
        """Predict state forward by dt seconds."""
        # State transition: constant velocity model
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Process noise: acceleration as noise source
        q = self.process_noise_std ** 2
        Q = np.zeros((6, 6))
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        # Position-position block
        Q[0, 0] = dt4 / 4 * q
        Q[1, 1] = dt4 / 4 * q
        Q[2, 2] = dt4 / 4 * q
        # Position-velocity cross block
        Q[0, 3] = dt3 / 2 * q
        Q[1, 4] = dt3 / 2 * q
        Q[2, 5] = dt3 / 2 * q
        Q[3, 0] = dt3 / 2 * q
        Q[4, 1] = dt3 / 2 * q
        Q[5, 2] = dt3 / 2 * q
        # Velocity-velocity block
        Q[3, 3] = dt2 * q
        Q[4, 4] = dt2 * q
        Q[5, 5] = dt2 * q

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, measurement):
        """Update state with a 3D position measurement [x, y, z]."""
        z = np.array(measurement)
        y = z - self.H @ self.x  # innovation
        S = self.H @ self.P @ self.H.T + self.R  # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    @property
    def position(self):
        return self.x[:3].copy()

    @property
    def velocity(self):
        return self.x[3:].copy()
