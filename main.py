import gyro_and_accel
import altimeter
import servo
import os
import time
import json
import numpy as np

def init():
    global pitch_filter, yaw_filter, json_start_time, datetime, dt
    os.makedirs(os.path.expanduser("~/Documents/Cocket Rocket/data_recordings"), exist_ok=True)
    datetime = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime())
    json_start_time = time.time()

    dt = 0.0 # Time step
    process_noise = 0.01
    measurement_noise = 0.1

    pitch_filter = gyro_and_accel.KalmanFilter(dt, process_noise, measurement_noise)
    yaw_filter = gyro_and_accel.KalmanFilter(dt, process_noise, measurement_noise)

def json_write(pitch, yaw, altitude, accelerometer_data, gyroscope_data, start_time, datetime):
    new_data_name = f"{time.time() - start_time}"
    new_data = {
        "pitch": pitch,
        "yaw": yaw,
        "altitude": altitude,
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

def rocket_motor_vectoring(rotation: tuple, goal_rotation: tuple):
    rocket_pitch, rocket_yaw = rotation
    goal_pitch, goal_yaw = goal_rotation
    dpitch = goal_pitch - rocket_pitch
    dyaw = goal_yaw - rocket_yaw
    if dpitch >= 15:
        servo.servo_write_angle(10, "X")
    if dpitch <= -15:
        servo.servo_write_angle(-10, "X")
    else:
        servo.servo_write_angle(dpitch*(2/3), "X")

    if dyaw >= 15:
        servo.servo_write_angle(-10, "Y")
    if dyaw <= -15:
        servo.servo_write_angle(10, "Y")
    else:
        servo.servo_write_angle(dyaw*(-2/3), "Y")

def main():
    init()
    print_count = 0

    while True:
        # Measure start time of loop
        start_time = time.time()

        # Count to only print and store to json five times a second
        print_count += 1

        #Read data
        accelerometer_data, gyroscope_data = gyro_and_accel.read_sensor_data()
        altitude = altimeter.measure_altitude()

        #Calculate pitch and yaw from accelerometer data
        acc_pitch, acc_yaw = gyro_and_accel.accel_pitch_yaw(accelerometer_data)

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
            # Control rocket motor vectoring
            rocket_motor_vectoring((pitch, yaw), (0, 0))
            print(f"Pitch: {pitch}°, Yaw: {yaw}°, Altitude: {altitude}m")
            json_write(pitch, yaw, altitude, accelerometer_data, gyroscope_data, json_start_time, datetime)

        while time.time() - start_time < dt:
            time.sleep(0.001)
            pass

if __name__ == "__main__":
    main()