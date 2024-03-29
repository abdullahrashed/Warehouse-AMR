#!/usr/bin/env python

import rospy
from mpu6050.MPU6050 import MPU6050
from hmc5883l.HMC5883L import HMC5883L
from sensor_msgs.msg import Imu

class imuPublisher:
    """
    Class for publishing IMU data.

    Args:
        rate (float): The publishing rate in Hz.
        biais_gz (float): Bias for the gyroscope's z-axis.
        to_degree_per_second_gz (float): Conversion factor for gyroscope's z-axis from raw data to degrees per second.

    Attributes:
        node_name (str): Name of the ROS node.
        sensor_mpu (MPU6050): Instance of the MPU6050 sensor.
        sensor_hmc (HMC5883L): Instance of the HMC5883L sensor.
        imu_pub (rospy.Publisher): Publisher for IMU data.
        rate (float): The publishing rate in Hz.
        biais_gz (float): Bias for the gyroscope's z-axis.
        to_degree_per_second_gz (float): Conversion factor for gyroscope's z-axis from raw data to degrees per second.
    """

    def __init__(self, rate, biais_gz, to_degree_per_second_gz):
        rospy.init_node("robo_imu")
        self.node_name = rospy.get_name()
        rospy.loginfo("IMU publisher  %s started" % self.node_name)

        self.sensor_mpu = MPU6050()
        self.sensor_hmc = HMC5883L(gauss=4.7, declination=(-2, 5))
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.rate = rate
        self.biais_gz = biais_gz
        self.to_degree_per_second_gz = to_degree_per_second_gz

    def publishImu(self, data_mpu, data_hmc):
        """
        Publishes IMU data.

        Args:
            data_mpu (tuple): Tuple containing accelerometer and gyroscope data from MPU6050 sensor.
            data_hmc (tuple): Tuple containing magnetometer data from HMC5883L sensor.
        """
        ax, ay, az, gx, gy, gz = data_mpu
        mx, my, mz = data_hmc
        now = rospy.Time.now()

        imu = Imu(header=rospy.Header(frame_id="imu_link"))
        imu.header.stamp = now
        imu.orientation.x = mx
        imu.orientation.y = my
        imu.orientation.z = mz
        imu.orientation.w = 0
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz * self.to_degree_per_second_gz - self.biais_gz

        self.imu_pub.publish(imu)

    def run(self):
        """
        Runs the IMU publisher.
        """
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            data_mpu = self.sensor_mpu.convert()
            data_hmc = self.sensor_hmc.read_data()
            self.publishImu(data_mpu, data_hmc)
            r.sleep()


if __name__ == '__main__':
    rate = rospy.get_param("rate", 50)
    biais_gz = rospy.get_param("biais_gz", 0.0)
    to_degree_per_second_gz = rospy.get_param("to_degree_per_second_gz", 1/14.0)
    imu_publisher = imuPublisher(rate, biais_gz, to_degree_per_second_gz)
    imu_publisher.run()
