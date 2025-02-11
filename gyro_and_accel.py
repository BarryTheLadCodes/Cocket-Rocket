import math
import mpu6050 # type: ignore
import time
import json
import os
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
    accelerometer_data = mpu6050.get_accel_data()
    gyroscope_data = mpu6050.get_gyro_data()
    return accelerometer_data, gyroscope_data

def accel_pitch_roll(accelerometer_data):
    ax = accelerometer_data['x']
    ay = accelerometer_data['y']
    az = accelerometer_data['z']

    pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
    roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return pitch, roll

def json_write(pitch, roll, accelerometer_data, gyroscope_data, start_time, datetime):
    new_data_name = f"{time.time() - start_time}"
    new_data = {
        "pitch": pitch,
        "roll": roll,
        "ax": accelerometer_data['x'],
        "ay": accelerometer_data['y'],
        "az": accelerometer_data['z'],
        "gx": gyroscope_data['x'],
        "gy": gyroscope_data['y'],
        "gz": gyroscope_data['z']
    }
    file_path = os.path.expanduser(f"~/Documents/Cocket Rocket/data_recordings/{datetime}.json")

    if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
        with open(file_path, "r") as file:
            data = json.load(file)
    else:
        data = {}
    
    data[new_data_name] = new_data
    with open(file_path, "w") as file:
        json.dump(data, file, indent=2)

def main():
    os.makedirs(os.path.expanduser("~/Documents/Cocket Rocket/data_recordings"), exist_ok=True)
    start_time = time.time()
    datetime = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime())

    dt = 0.001 # Time step
    process_noise = 0.01
    measurement_noise = 0.1

    pitch_filter = KalmanFilter(dt, process_noise, measurement_noise)
    roll_filter = KalmanFilter(dt, process_noise, measurement_noise)

    while True:
        #Read data
        accelerometer_data, gyroscope_data = read_sensor_data()

        #Calculate pitch and roll from accelerometer data
        acc_pitch, acc_roll = accel_pitch_roll(accelerometer_data)

        # Predict using gyroscope data
        pitch_filter.predict(gyroscope_data['x'])
        roll_filter.predict(gyroscope_data['y'])

        # Update using accelerometer data
        pitch_filter.update(np.array([[acc_pitch]]))
        roll_filter.update(np.array([[acc_roll]]))

        # Get filtered pitch and roll
        pitch = pitch_filter.get_state()
        roll = roll_filter.get_state()

        print(f"Pitch: {pitch}, Roll: {roll}")

        # Write data to json file
        json_write(pitch, roll, accelerometer_data, gyroscope_data, start_time, datetime)

        time.sleep(dt)

if __name__ == "__main__":
    main()