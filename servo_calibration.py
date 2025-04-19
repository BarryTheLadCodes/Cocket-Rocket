import pigpio # type: ignore
import time

# Set pin 21 as servo1 output
SERVO_1_PIN = 21
SERVO_2_PIN = 20
servo_1 = pigpio.pi('localhost', 8888)
servo_2 = pigpio.pi('localhost', 8888)

if not servo_1.connected or not servo_2.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

servo_1.set_PWM_frequency(SERVO_1_PIN, 50)  # Set frequency to 50Hz
servo_1.set_servo_pulsewidth(SERVO_1_PIN, 1500) # Set initial pulse width to 1.5ms (neutral position)
servo_2.set_PWM_frequency(SERVO_2_PIN, 50)  # Set frequency to 50Hz
servo_2.set_servo_pulsewidth(SERVO_2_PIN, 1500) # Set initial pulse width to 1.5ms (neutral position)

def servo_write_angle(angle, servo):
    min_pulse = 1000
    max_pulse = 2000
    # Convert angle from -90 to 90 degrees to 0 to 180 degrees
    angle = angle*2 + 90
    if servo == 1:
        servo_1.set_servo_pulsewidth(SERVO_1_PIN, (angle / 180) * (max_pulse - min_pulse) + min_pulse)
    elif servo == 2:
        servo_2.set_servo_pulsewidth(SERVO_2_PIN, (angle / 180) * (max_pulse - min_pulse) + min_pulse)

while True:
    angle1 = input("Enter test angle for servo 1 (-90 to 90): ")
    angle2 = input("Enter test angle for servo 2 (-90 to 90): ")
    try:
        angle1 = float(angle1)
        angle2 = float(angle2)
        if -90 <= angle1 <= 90 and -90 <= angle2 <= 90:
            servo_write_angle(angle1, 1)
            servo_write_angle(angle2, 2)
            time.sleep(0.1)
        else:
            print("Angles must be between -90 and 90 degrees.")
    except ValueError:
        print("Invalid input. Please enter a number between -90 and 90.")