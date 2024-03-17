#!/usr/bin/env python

# reference: https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer

import smbus
import time

class MPU6050:
    def __init__(self):
        """
        Initializes the MPU6050 class.

        Attributes:
        - MPU6050_ADDR: The I2C address of the MPU6050 sensor.
        - PWR_MGMT_1: Register address for power management.
        - SMPLRT_DIV: Register address for sample rate divider.
        - CONFIG: Register address for configuration.
        - GYRO_CONFIG: Register address for gyroscope configuration.
        - ACCEL_CONFIG: Register address for accelerometer configuration.
        - INT_ENABLE: Register address for interrupt enable.
        - ACCEL_XOUT_H: Register address for accelerometer X-axis high byte.
        - ACCEL_YOUT_H: Register address for accelerometer Y-axis high byte.
        - ACCEL_ZOUT_H: Register address for accelerometer Z-axis high byte.
        - TEMP_OUT_H: Register address for temperature sensor high byte.
        - GYRO_XOUT_H: Register address for gyroscope X-axis high byte.
        - GYRO_YOUT_H: Register address for gyroscope Y-axis high byte.
        - GYRO_ZOUT_H: Register address for gyroscope Z-axis high byte.
        - bus: The SMBus object for I2C communication.
        - gyro_sens: The sensitivity of the gyroscope.
        - accel_sens: The sensitivity of the accelerometer.
        """
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.TEMP_OUT_H   = 0x41
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        self.bus = smbus.SMBus(1)
        self.gyro_sens = None
        self.accel_sens = None

    def start(self):
        """
        Starts the MPU6050 sensor by configuring its registers.

        This method sets the sample rate, resets the sensors, configures power management and crystal settings,
        and sets the gyroscope and accelerometer configurations.

        Args:
            None

        Returns:
            None
        """
        # alter sample rate (stability)
        samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, samp_rate_div)
        time.sleep(0.1)
        # reset all sensors
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # power management and crystal settings
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        #Write to Configuration register
        self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0)
        time.sleep(0.1)
        #Write to Gyro configuration register
        gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
        gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
        gyro_indx = 0
        self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
        time.sleep(0.1)
        #Write to Accel configuration register
        accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
        accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
        accel_indx = 0                            
        self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
        time.sleep(0.1)
        # interrupt register (related to overflow of data [FIFO])
        self.bus.write_byte_data(self.MPU6050_ADDR, self.INT_ENABLE, 1)
        time.sleep(0.1)
        self.gyro_sens = gyro_config_vals[gyro_indx]
        self.accel_sens = accel_config_vals[accel_indx]

    def read_raw_bits(self, register):
        """
        Reads the raw bits from the specified register of the MPU6050 sensor.

        Args:
            register (int): The register address to read from.

        Returns:
            int: The raw bits read from the register.
        """
        # read accel and gyro values
        high = self.bus.read_byte_data(self.MPU6050_ADDR, register)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, register+1)

        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)
        
        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

    def convert(self):
        """
        Converts the raw sensor readings to acceleration and gyroscope values.

        This method reads the raw bits from the accelerometer and gyroscope registers,
        and converts them to acceleration in g and gyroscope values in degrees per second.

        Args:
            None

        Returns:
            tuple: A tuple containing the acceleration and gyroscope values in the order (ax, ay, az, gx, gy, gz).
        """
        # raw acceleration bits
        acc_x = self.read_raw_bits(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_bits(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_bits(self.ACCEL_ZOUT_H)

        # raw temp bits
        # t_val = self.read_raw_bits(self.TEMP_OUT_H) # uncomment to read temp
        
        # raw gyroscope bits
        gyro_x = self.read_raw_bits(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_bits(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_bits(self.GYRO_ZOUT_H)

        #convert to acceleration in g and gyro dps
        ax = (acc_x/(2.0**15.0))*self.accel_sens
        ay = (acc_y/(2.0**15.0))*self.accel_sens
        az = (acc_z/(2.0**15.0))*self.accel_sens

        gx = (gyro_x/(2.0**15.0))*self.gyro_sens
        gy = (gyro_y/(2.0**15.0))*self.gyro_sens
        gz = (gyro_z/(2.0**15.0))*self.gyro_sens

        # temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
        return ax, ay, az, gx, gy, gz

if __name__ == "__main__":
    mpu6050 = MPU6050()
    mpu6050.start()
    ax, ay, az, gx, gy, gz = mpu6050.convert()
    while True:
        print("Accel: ", ax, ay, az)
        print("Gyro: ", gx, gy, gz)
        time.sleep(5)
