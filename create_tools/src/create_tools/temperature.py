#!/usr/bin/env python3

import smbus
import rospy
import time
from sensor_msgs.msg import Temperature, Imu
from registers import (
    PWR_MGMT_1,
    TEMP_H,
)
from device_handler import DeviceHandler

DEFAULT_ADDRESS = 0x68


class TemperatureHandler(DeviceHandler):
    def __init__(self, bus, hz, address=DEFAULT_ADDRESS):
        super().__init__(bus, address, hz)
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")
        self.temp_pub = rospy.Publisher("imu/temp", Temperature, queue_size=5)

    def init_device(self):
        rospy.loginfo("Initializing Temperature Sensor")
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
        time.sleep(0.1)

    def execute(self):
        if super().execute():
            self.publish_temp()

    def publish_temp(self):
        temp_msg = Temperature()
        temp_msg.header.frame_id = self.imu_frame
        temp_msg.temperature = self.read_word_2c(TEMP_H) / 340.0 + 36.53
        temp_msg.header.stamp = rospy.Time.now()
        self.temp_pub.publish(temp_msg)
