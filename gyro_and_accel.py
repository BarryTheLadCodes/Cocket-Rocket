import math
import mpu6050 # type: ignore
import time
import numpy as np

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

class KalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise):
        self.dt = dt  # Time step
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        
        # State transition matrix
        self.A = np.array([[1, dt], [0, 1]], dtype=np.float64)

        self.B = np.array([[dt], [dt]], dtype=np.float64)
        
        # Measurement matrix
        self.H = np.array([[1, 0]], dtype=np.float64)
        
        # Process noise covariance
        self.Q = np.array([[process_noise, 0], [0, process_noise]], dtype=np.float64)
        
        # Measurement noise covariance
        self.R = np.array([[measurement_noise]], dtype=np.float64)
        
        # State estimate initialized as float64
        self.x = np.array([[0.0], [0.0]], dtype=np.float64)  # Ensuring it's float64
        
        # Estimate covariance
        self.P = np.eye(2, dtype=np.float64)
    
    def predict(self, gyroscope_data):
        # Update the angle and angular velocity prediction using gyroscope data
        self.x = self.A @ self.x + self.B * gyroscope_data 
        # Predict covariance
        self.P = self.A @ self.P @ self.A.T + self.Q
    
    def update(self, acc):
        # Calculate Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        # Update state estimate
        self.x = self.x + K @ (acc - self.H @ self.x)
        # Update covariance
        self.P = (np.eye(2) - K @ self.H) @ self.P
    
    def get_state(self):
        return self.x[0, 0]

# Define a function to read the sensor data
def read_sensor_data():
    start = time.monotonic()
    accelerometer_data = mpu6050.get_accel_data()
    gyroscope_data = mpu6050.get_gyro_data()
    print(f"MPU Time: {time.monotonic() - start}")
    return accelerometer_data, gyroscope_data

def accel_pitch_roll(accelerometer_data):
    ax = accelerometer_data['x']
    ay = accelerometer_data['y']
    az = accelerometer_data['z']

    pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
    roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return pitch, roll