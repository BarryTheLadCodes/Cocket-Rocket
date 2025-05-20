import time
import servo

while True:
    for i in range(3):
        servo.servo_write_angle(-10, "X")
        servo.servo_write_angle(-10+i*10, "Y")
        time.sleep(3)
    for i in range(3):
        servo.servo_write_angle(0, "X")
        servo.servo_write_angle(10-i*10, "Y")
        time.sleep(3)
    for i in range(3):
        servo.servo_write_angle(10, "X")
        servo.servo_write_angle(-10+i*10, "Y")
        time.sleep(3)