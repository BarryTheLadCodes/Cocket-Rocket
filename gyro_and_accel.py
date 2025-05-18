import math
import mpu6050 # type: ignore
import time
import numpy as np

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)
#Set sample rate to 1kHz
#mpu6050.bus.write_byte_data(mpu6050.address, 0x19, 0x00)
#Disable the digital low-pass filter for lower latency
mpu6050.bus.write_byte_data(mpu6050.address, 0x1A, 0x00)
#Set accelerometer range to +-2g
mpu6050.bus.write_byte_data(mpu6050.address, 0x1C, 0x00)
#Set gyroscope range to +-250 degrees per second
mpu6050.bus.write_byte_data(mpu6050.address, 0x1B, 0x00)

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
    # Read raw data directly from i2c as a block to make it more efficient
    raw_data = mpu6050.bus.read_i2c_block_data(mpu6050.address, 0x3B, 14)  # Read 14 bytes at once
    ax = (raw_data[0] << 8) | raw_data[1]
    ay = (raw_data[2] << 8) | raw_data[3]
    az = (raw_data[4] << 8) | raw_data[5]
    gx = (raw_data[8] << 8) | raw_data[9]
    gy = (raw_data[10] << 8) | raw_data[11]
    gz = (raw_data[12] << 8) | raw_data[13]

    # Handling two's complement for signed values
    ax = ax - 65536 if ax > 32767 else ax
    ay = ay - 65536 if ay > 32767 else ay
    az = az - 65536 if az > 32767 else az
    gx = gx - 65536 if gx > 32767 else gx
    gy = gy - 65536 if gy > 32767 else gy
    gz = gz - 65536 if gz > 32767 else gz
    
    #Convert to m/s^2 and °/s
    ax = (ax / 16384) * 9.80665
    ay = (ay / 16384) * 9.80665
    az = (az / 16384) * 9.80665
    gx = (gx / 131)
    gy = (gy / 131)
    gz = (gz / 131)

    accelerometer_data = {
        'x': ax,
        'y': ay,
        'z': az
    }
    gyroscope_data = {
        'x': gx,
        'y': gy,
        'z': gz
    }
    return accelerometer_data, gyroscope_data

def accel_pitch_yaw(accelerometer_data):
    ax = accelerometer_data['x']
    ay = accelerometer_data['y']
    az = accelerometer_data['z']

    pitch = math.atan2(ax, ay) * 180 / math.pi
    yaw = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return pitch, yaw

if __name__ == "__main__":
    print_count = 0
    dt = 0.02 # Time step
    process_noise = 0.01
    measurement_noise = 0.1

    pitch_filter = KalmanFilter(dt, process_noise, measurement_noise)
    yaw_filter = KalmanFilter(dt, process_noise, measurement_noise)

    while True:
        # Measure start time of loop
        start_time = time.time()

        # Count to only print and store to json five times a second
        print_count += 1

        #Read data
        accelerometer_data, gyroscope_data = read_sensor_data()

        #Calculate pitch and yaw from accelerometer data
        acc_pitch, acc_yaw = accel_pitch_yaw(accelerometer_data)

        # Predict using gyroscope data
        pitch_filter.predict(gyroscope_data['x'])
        yaw_filter.predict(gyroscope_data['y'])

        # Update using accelerometer data
        pitch_filter.update(np.array([[acc_pitch]]))
        yaw_filter.update(np.array([[acc_yaw]]))

        # Get filtered pitch and yaw
        pitch = pitch_filter.get_state()
        yaw = yaw_filter.get_state()

        if print_count == 10:
            print_count = 0
            print(f"Pitch: {pitch}°, Yaw: {yaw}°")

        while time.time() - start_time < dt:
            time.sleep(0.001)
            pass