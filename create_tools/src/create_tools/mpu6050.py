import smbus
import math
import time
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
from device_handler import DeviceHandler

DEFAULT_ADDRESS = 0x68

# Based on https://github.com/OSUrobotics/mpu_6050_driver
# by OSURobotics


class MPU6050Handler(DeviceHandler):
    def __init__(self, bus, hz, address=DEFAULT_ADDRESS):
        super().__init__(bus, hz, address)
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")
        self.temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size=5)
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=5)

    def init_device(self):
        rospy.loginfo(f"Initializing MPU6050 at address: {self.address}")
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
        time.sleep(0.1)
        self.calibrate()

    def execute(self):
        if super().execute():
            rospy.logdebug("Publishing IMU data")
            self.publish_imu()

    def calibrate(self, n_samples=100):
        self.gyro_offset = self.sample_gyro(n_samples)
        self.accel_offset = self.sample_accel(n_samples)
        rospy.loginfo("Gyro offset: {}".format(self.gyro_offset))
        rospy.loginfo("Accel offset: {}".format(self.accel_offset))

    def sample_gyro(self, n_samples=100):
        ret = []
        for i in range(n_samples):
            gyro_x, gyro_y, gyro_z = self.read_gyro()
            ret.append([gyro_x, gyro_y, gyro_z])
        return np.mean(ret, axis=0)

    def sample_accel(self, n_samples=100):
        ret = []
        for i in range(n_samples):
            accel_x, accel_y, accel_z = self.read_accel()
            ret.append([accel_x, accel_y, accel_z])
        return np.mean(ret, axis=0)

    def read_gyro(self):
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0
        return gyro_x, gyro_y, gyro_z

    def read_accel(self):
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0
        return accel_x, accel_y, accel_z

    def publish_imu(self):
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

        imu_msg.linear_acceleration.x = accel_x - self.accel_offset[0]
        imu_msg.linear_acceleration.y = accel_y - self.accel_offset[1]
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = math.radians(gyro_x - self.gyro_offset[0])
        imu_msg.angular_velocity.y = math.radians(gyro_y - self.gyro_offset[1])
        imu_msg.angular_velocity.z = math.radians(gyro_z - self.gyro_offset[2])

        imu_msg.header.stamp = rospy.Time.now()

        self.imu_pub.publish(imu_msg)
