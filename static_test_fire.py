import time
import servo

while True:
    servo.servo_write_angle(0, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.5)
    servo.servo_write_angle(-10, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.5)
    servo.servo_write_angle(10, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.5)   
    servo.servo_write_angle(0, "X")
    servo.servo_write_angle(0, "Y")
    time.sleep(0.5)
    servo.servo_write_angle(0, "X")
    servo.servo_write_angle(-10, "Y")
    time.sleep(0.5)
    servo.servo_write_angle(0, "X")
    servo.servo_write_angle(10, "Y")
    time.sleep(0.5)