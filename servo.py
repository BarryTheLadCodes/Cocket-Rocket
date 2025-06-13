import pigpio # type: ignore
import time

# Set pin 21 as servo1 output
SERVO_X_PIN = 20
SERVO_Y_PIN = 21
SERVO_X_OFFSET = 0
SERVO_Y_OFFSET = 0
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

if __name__ == "__main__":
    while True:
        angleX = input("Enter test angle for servo X (-90 to 90): ")
        angleY = input("Enter test angle for servo Y (-90 to 90): ")
        try:
            angleX = float(angleX)
            angleY = float(angleY)
            if -90 <= angleX <= 90 and -90 <= angleY <= 90:
                servo_write_angle(angleX, "X")
                servo_write_angle(angleY, "Y")
                time.sleep(0.1)
            else:
                print("Angles must be between -90 and 90 degrees.")
        except ValueError:
            print("Invalid input. Please enter a number between -90 and 90.")