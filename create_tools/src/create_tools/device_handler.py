import smbus
import rospy
from abc import ABC, abstractmethod


class DeviceHandler(ABC):
    def __init__(self, bus, hz, address):
        self.bus = bus
        self.address = address
        if type(self.address) == str:
            self.address = int(self.address, 16)
        self.last_time = rospy.rostime.get_rostime()
        self.sleep_dur = rospy.rostime.Duration(0, int(1e9 / hz))

    @abstractmethod
    def init_device(self):
        pass

    def should_execute(self):
        return rospy.rostime.get_rostime() - self.last_time > self.sleep_dur

    @abstractmethod
    def execute(self):
        if self.should_execute():
            self.last_time = rospy.rostime.get_rostime()
            return True
        else:
            return False

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
