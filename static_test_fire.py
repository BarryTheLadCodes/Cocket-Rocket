import time
import servo

while True:
    servo.servo_write_angle(0, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.25)
    servo.servo_write_angle(-15, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.25)
    servo.servo_write_angle(15, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.25)