#!/usr/bin/env python3

import smbus
import rospy
from mpu6050 import MPU6050Handler
from battery import BatteryHandler
from temperature import TemperatureHandler

MAXIMUM_RATE = 500


class I2CBusNode:
    def __init__(self):
        self.bus = smbus.SMBus(rospy.get_param("~bus", 1))
        self.handlers = [
            MPU6050Handler(self.bus, 100),
            BatteryHandler(self.bus, 5),
            TemperatureHandler(self.bus, 5),
        ]

    def init_devices(self):
        for handler in self.handlers:
            handler.init_device()

    def execute(self):
        for handler in self.handlers:
            handler.execute()


if __name__ == "__main__":
    rospy.init_node("i2c_bus_node")
    node = I2CBusNode()
    node.init_devices()
    r = rospy.Rate(MAXIMUM_RATE)
    while not rospy.is_shutdown():
        r.sleep()
        node.execute()
