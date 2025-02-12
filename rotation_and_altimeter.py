import smbus # type: ignore
import time
import gyro_and_accel

BUS = smbus.SMBus(1)  # Use I2C bus 1
MPU6050_ADDR = 0x68   # MPU6050 I2C address

def setup_bypass():
    # Write 0x00 to PWR_MGMT_1 to wake up the MPU6050
    BUS.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
    time.sleep(0.1)

    # Write 0x02 to INT_PIN_CFG to enable bypass mode
    BUS.write_byte_data(MPU6050_ADDR, 0x37, 0x02)

    # Write 0x00 to USER_CTRL to disable MPU6050 master mode
    BUS.write_byte_data(MPU6050_ADDR, 0x6A, 0x00)

    print("Bypass Activated")

setup_bypass()