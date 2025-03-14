import pigpio # type: ignore
import time

# Set pin 21 as servo1 output
SERVO_1_PIN = 21
servo_1 = pigpio.pi()

if not servo_1.connected:
    print("Failed to connect to pigpio daemon!")
    exit()

servo_1.set_PWM_frequency(SERVO_1_PIN, 50)  # Set frequency to 50Hz
servo_1.set_servo_pulsewidth(SERVO_1_PIN, 1500) # Set initial pulse width to 1.5ms (neutral position)

def servo_write_angle(angle):
    min_pulse = 1000
    max_pulse = 2000
    # Convert angle from -90 to 90 degrees to 0 to 180 degrees
    angle = angle + 90
    servo_1.set_servo_pulsewidth(SERVO_1_PIN, (angle / 180) * (max_pulse - min_pulse) + min_pulse)
while True:
    servo_write_angle(-90)
    time.sleep(1)
    servo_write_angle(90)
    time.sleep(1)