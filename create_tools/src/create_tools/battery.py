import struct
import rospy
from std_msgs.msg import Float32
from device_handler import DeviceHandler


DEFAULT_ADDRESS = 0x36


class BatteryHandler(DeviceHandler):
    def __init__(self, bus, hz, address=DEFAULT_ADDRESS):
        super().__init__(bus, address, hz)
        self.cap_pub = rospy.Publisher("battery/capacity", Float32, queue_size=5)
        self.volt_pub = rospy.Publisher("battery/voltage", Float32, queue_size=5)

    def init_device(self):
        rospy.loginfo("Initializing Battery")

    def execute(self):
        if super().execute():
            rospy.loginfo("Publishing battery data")
            self.last_time = rospy.rostime.get_rostime()
            voltage = self.read_voltage()
            capacity = self.read_capacity()
            self.cap_pub.publish(Float32(capacity))
            self.volt_pub.publish(Float32(voltage))

    def read_voltage(self):
        read = self.bus.read_word_data(self.address, 2)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        voltage = swapped * 1.25 / 1000 / 16
        return voltage

    def read_capacity(self):
        read = self.bus.read_word_data(self.address, 4)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        capacity = swapped / 256
        return capacity
