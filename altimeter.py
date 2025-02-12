import smbus # type: ignore
import busio # type: ignore
import board # type: ignore
import adafruit_lps2x # type: ignore
import time

def setup_bypass():
    BUS = smbus.SMBus(1)  # Use I2C bus 1
    MPU6050_ADDR = 0x68   # MPU6050 I2C address
    # Write 0x00 to PWR_MGMT_1 to wake up the MPU6050
    BUS.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
    time.sleep(0.1)

    # Write 0x02 to INT_PIN_CFG to enable bypass mode
    BUS.write_byte_data(MPU6050_ADDR, 0x37, 0x02)

    # Write 0x00 to USER_CTRL to disable MPU6050 master mode
    BUS.write_byte_data(MPU6050_ADDR, 0x6A, 0x00)

    print("Bypass Activated")

lps_i2c = busio.I2C(board.SCL, board.SDA)
lps22 = adafruit_lps2x.LPS22(lps_i2c, address=0x5C)

def read_sensor_data():
    pressure = lps22.pressure
    return pressure

setup_bypass()
print(read_sensor_data())