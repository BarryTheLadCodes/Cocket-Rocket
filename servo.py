import pigpio # type: ignore
import time

# Set pin 21 as servo1 output
SERVO_X_PIN = 20
SERVO_Y_PIN = 21
SERVO_X_OFFSET = 10
SERVO_Y_OFFSET = -5
servo_X = pigpio.pi('localhost', 8888)
servo_Y = pigpio.pi('localhost', 8888)

if not servo_X.connected or not servo_X.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

servo_X.set_PWM_frequency(SERVO_X_PIN, 50)  # Set frequency to 50Hz
servo_X.set_servo_pulsewidth(SERVO_X_PIN, 1500) # Set initial pulse width to 1.5ms (neutral position)
servo_Y.set_PWM_frequency(SERVO_Y_PIN, 50)  # Set frequency to 50Hz
servo_Y.set_servo_pulsewidth(SERVO_Y_PIN, 1500) # Set initial pulse width to 1.5ms (neutral position)

def servo_write_angle(angle, servo):
    min_pulse = 1000
    max_pulse = 2000
    # Convert angle from -90 to 90 degrees to 0 to 180 degrees
    if servo == "X":
        angle = angle + SERVO_X_OFFSET + 90
        servo_X.set_servo_pulsewidth(SERVO_X_PIN, (angle / 180) * (max_pulse - min_pulse) + min_pulse)
    elif servo == "Y":
        angle = angle + SERVO_Y_OFFSET + 90
        servo_Y.set_servo_pulsewidth(SERVO_Y_PIN, (angle / 180) * (max_pulse - min_pulse) + min_pulse)

while True:
    servo_write_angle(-8.5, "X")
    servo_write_angle(8.5, "Y")
    time.sleep(1)
    servo_write_angle(0, "X")
    servo_write_angle(0, "Y")
    time.sleep(1)
    servo_write_angle(8.5, "X")
    servo_write_angle(-8.5, "Y")
    time.sleep(1)