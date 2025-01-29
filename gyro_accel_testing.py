import math
import mpu6050
import time

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

delay = 0.01
alpha = 0.98 #Fliter constant, determines split between accelerometer and gyroscope data
pitch_angle = 0
roll_angle = 0
yaw_angle = 0

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()
    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()
    return accelerometer_data, gyroscope_data

def angles():
    accelerometer_data, gyroscope_data = read_sensor_data()
    #Get angular velocities
    pitch_data = gyroscope_data['x']
    roll_data = gyroscope_data['y']
    yaw_data = gyroscope_data['z']

    #Get accelerations
    accelerometer_x = accelerometer_data['x']
    accelerometer_y = accelerometer_data['y']
    accelerometer_z = accelerometer_data['z']

    #Calculate pitch and roll estimations using accelerometer data
    pitch_angle_accelerometer = math.atan(accelerometer_y/math.sqrt(accelerometer_x**2 + accelerometer_z**2))
    roll_angle_accelerometer = math.atan(accelerometer_x/math.sqrt(accelerometer_y**2 + accelerometer_z**2))

    #Combine accelerometer and gyroscope data for final angle
    pitch_angle = alpha * (pitch_angle + gyroscope_data['x'] * delay) + (1 - alpha) * pitch_angle_accelerometer
    roll_angle = alpha * (roll_angle + gyroscope_data['y'] * delay) + (1 - alpha) * roll_angle_accelerometer

    #Integrate yaw angular velocity with time to get yaw angle approximation
    yaw_angle += yaw_data * delay

    return pitch_angle, roll_angle, yaw_angle

# Start a while loop to continuously read the sensor data
while True:
    # Read the sensor data
    pitch_angle, roll_angle, yaw_angle = angles()
    # Print the sensor data
    print("Pitch: ", pitch_angle)
    print("Roll: ", roll_angle)
    print("Yaw: ", yaw_angle)
    
    time.sleep(delay)