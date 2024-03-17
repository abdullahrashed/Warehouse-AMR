#!/usr/bin/env python

# HMC5888L Magnetometer (Digital Compass) wrapper class
# Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
# but uses smbus rather than quick2wire and sets some different init
# params.

import smbus
import math
import time


class HMC5883L:
    """
    HMC5883L.py is a wrapper class for the HMC5888L Magnetometer (Digital Compass).
    It provides methods to read data from the magnetometer and calculate the heading.

    Attributes:
        bus (smbus.SMBus): The SMBus object for communication with the I2C bus.
        address (int): The I2C address of the magnetometer.
        __declDegrees (int): The degrees part of the declination angle.
        __declMinutes (int): The minutes part of the declination angle.
        __declination (float): The declination angle in radians.
        __scale (float): The scale factor for converting raw data to magnetic field values.

    Methods:
        __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0, 0)): Initializes the HMC5883L object.
        declination(self): Returns the declination angle.
        twos_complement(self, val, len): Converts a twos complement value to an integer.
        __convert(self, data, offset): Converts raw data to magnetic field values.
        read_data(self): Reads the magnetometer data.
        heading(self): Calculates the heading based on the magnetometer data.
        degrees(self, headingDeg): Converts the heading from degrees to degrees and minutes.
        __str__(self): Returns a string representation of the magnetometer data.

    Usage:
        compass = HMC5883L(gauss=4.7, declination=(-2, 5))
        while True:
            print(compass)
            time.sleep(5)
    """

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0, 0)):
        """
        Initializes the HMC5883L object.

        Args:
            port (int): The I2C port number.
            address (int): The I2C address of the magnetometer.
            gauss (float): The desired magnetic field range in gauss.
            declination (tuple): The declination angle in degrees and minutes.

        Returns:
            None
        """
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70)  # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5)  # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00)  # Continuous measurement

    def declination(self):
        """
        Returns the declination angle.

        Returns:
            tuple: The declination angle in degrees and minutes.
        """
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        """
        Converts a twos complement value to an integer.

        Args:
            val (int): The twos complement value.
            len (int): The number of bits in the value.

        Returns:
            int: The converted integer value.
        """
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1 << len)
        return val

    def __convert(self, data, offset):
        """
        Converts raw data to magnetic field values.

        Args:
            data (list): The raw data from the magnetometer.
            offset (int): The offset of the data in the list.

        Returns:
            float: The converted magnetic field value.
        """
        val = self.twos_complement(data[offset] << 8 | data[offset + 1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def read_data(self):
        """
        Reads the magnetometer data.

        Returns:
            tuple: The magnetometer data in the form (mx, my, mz).
        """
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        mx = self.__convert(data, 3)
        my = self.__convert(data, 7)
        mz = self.__convert(data, 5)
        return (mx, my, mz)

    def heading(self):
        """
        Calculates the heading based on the magnetometer data.

        Returns:
            float: The heading in degrees.
        """
        (mx, my, mz) = self.read_data()
        headingRad = math.atan2(my, mx)
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        """
        Converts the heading from degrees to degrees and minutes.

        Args:
            headingDeg (float): The heading in degrees.

        Returns:
            tuple: The heading in degrees and minutes.
        """
        degrees = math.floor(headingDeg[0])
        minutes = round((headingDeg[1] - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        """
        Returns a string representation of the magnetometer data.

        Returns:
            str: The string representation of the magnetometer data.
        """
        (mx, my, mz) = self.read_data()
        return "Axis X: " + str(mx) + "\n" + \
               "Axis Y: " + str(my) + "\n" + \
               "Axis Z: " + str(mz) + "\n" + \
               "Declination: " + str(self.degrees(self.declination())) + "\n" + \
               "Heading: " + str(self.heading()) + "\n"


if __name__ == "__main__":
    compass = HMC5883L(gauss=4.7, declination=(-2, 5))
    while True:
        print(compass)
        time.sleep(5)