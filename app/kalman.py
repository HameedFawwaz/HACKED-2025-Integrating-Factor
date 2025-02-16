import numpy as np

import numpy as np

class KalmanFilterIMU:
    def __init__(self, dt, process_noise, measurement_noise, error_covariance):
        self.dt = dt  # Time step

        # State vector [ax, ay, az]
        self.x = np.zeros((3, 1))
        
        # State transition matrix
        self.F = np.eye(3)
        
        # Process noise covariance
        self.Q = np.eye(3) * process_noise
        
        # Measurement matrix (we measure acceleration directly)
        self.H = np.eye(3)
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
        
        # Error covariance matrix
        self.P = np.eye(3) * error_covariance
    
    def predict(self):
        """
        Predict step assuming constant acceleration model.
        """
        # Predict state and error covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
    
    def update(self, accel):
        """
        Update step using accelerometer data.
        :param accel: Accelerometer readings [ax, ay, az].
        """
        z = np.array(accel).reshape((3, 1))  # Measurement vector
        
        # Kalman gain
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Update state and error covariance
        y = z - np.dot(self.H, self.x)  # Measurement residual
        self.x = self.x + np.dot(K, y)
        I = np.eye(3)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
    
    def get_filtered_acceleration(self):
        return self.x.flatten()  # ax, ay, a
# Example usage
"""dt = 0.01  # Time step (100 Hz update rate)
kf = KalmanFilterIMU(dt, process_noise=0.001, measurement_noise=0.01, error_covariance=1.0)

gyro_data = [0.01, -0.02]  # Example gyroscope readings in rad/s
accel_data = [0.0, 0.0, 9.81]  # Example accelerometer readings (gravity aligned)

kf.predict(gyro_data)
kf.update(accel_data)

roll, pitch = kf.get_estimated_angles()
print(f"Estimated Roll: {roll:.4f} rad, Pitch: {pitch:.4f} rad")
"""