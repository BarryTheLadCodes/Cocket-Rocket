import gyro_and_accel
import altimeter
import os
import time
import json
import numpy as np

def init():
    global pitch_filter, roll_filter, json_start_time, datetime, dt
    os.makedirs(os.path.expanduser("~/Documents/Cocket Rocket/data_recordings"), exist_ok=True)
    datetime = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime())
    json_start_time = time.time()

    dt = 0.02 # Time step
    process_noise = 0.01
    measurement_noise = 0.1

    pitch_filter = gyro_and_accel.KalmanFilter(dt, process_noise, measurement_noise)
    roll_filter = gyro_and_accel.KalmanFilter(dt, process_noise, measurement_noise)

def json_write(pitch, roll, altitude, accelerometer_data, gyroscope_data, start_time, datetime):
    new_data_name = f"{time.time() - start_time}"
    new_data = {
        "pitch": pitch,
        "roll": roll,
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

        #Calculate pitch and roll from accelerometer data
        acc_pitch, acc_roll = gyro_and_accel.accel_pitch_roll(accelerometer_data)

        # Predict using gyroscope data
        pitch_filter.predict(gyroscope_data['x'])
        roll_filter.predict(gyroscope_data['y'])

        # Update using accelerometer data
        pitch_filter.update(np.array([[acc_pitch]]))
        roll_filter.update(np.array([[acc_roll]]))

        # Get filtered pitch and roll
        pitch = pitch_filter.get_state()
        roll = roll_filter.get_state()

        if print_count == 10:
            print_count = 0
            print(f"Pitch: {pitch}°, Roll: {roll}°, Altitude: {altitude}m")
            json_write(pitch, roll, altitude, accelerometer_data, gyroscope_data, json_start_time, datetime)

        while time.time() - start_time < dt:
            time.sleep(0.001)
            pass

if __name__ == "__main__":
    main()