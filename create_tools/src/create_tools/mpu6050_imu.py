#!/usr/bin/env python3

import smbus
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis
from registers import (
    PWR_MGMT_1,
    ACCEL_XOUT_H,
    ACCEL_YOUT_H,
    ACCEL_ZOUT_H,
    TEMP_H,
    GYRO_XOUT_H,
    GYRO_YOUT_H,
    GYRO_ZOUT_H,
)

# Based on https://github.com/OSUrobotics/mpu_6050_driver
# by OSURobotics


class MPU6050:
    def __init__(self):
        self.bus = smbus.SMBus(rospy.get_param("~bus", 1))
        self.address = rospy.get_param("~device_address", 0x68)
        if type(self.address) == str:
            self.address = int(self.address, 16)

        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")

        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)

        self.temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size=5)
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=5)
        self.imu_timer = rospy.Timer(rospy.Duration(0.02), self.publish_imu)
        self.temp_timer = rospy.Timer(rospy.Duration(10), self.publish_temp)

    # read_word and read_word_2c from
    # http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
    def read_word(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, register):
        val = self.read_word(register)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_temp(self, timer_event):
        temp_msg = Temperature()
        temp_msg.header.frame_id = self.imu_frame
        temp_msg.temperature = self.read_word_2c(TEMP_H) / 340.0 + 36.53
        temp_msg.header.stamp = rospy.Time.now()
        self.temp_pub.publish(temp_msg)

    def publish_imu(self, timer_event):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.imu_frame

        # Read the acceleration vals
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0

        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read the gyro vals
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0

        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = rospy.Time.now()

        self.imu_pub.publish(imu_msg)


if __name__ == "__main__":
    rospy.init_node("imu_node")
    mpu = MPU6050()
    rospy.spin()
