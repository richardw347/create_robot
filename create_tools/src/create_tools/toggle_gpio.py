#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO


PIN = 18
JOY_BUTTON = 5
SAFETY_TIMER = 2


class ToggleGPIO:
    def __init__(self):
        self.pin = PIN
        self.joy_button = JOY_BUTTON
        self.timer = None
        self.joy_sub = rospy.Subscriber("joy", Joy, self.handle_joy_msg)
        self.service = rospy.Service(
            "toggle_gpio", SetBool, self.handle_toggle_pin_service
        )

    def toggle_pin(self, pin, state):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        if state:
            GPIO.output(pin, GPIO.HIGH)
            if self.timer is not None:
                self.timer = rospy.Timer(
                    rospy.Duration(SAFETY_TIMER), self.handle_timer_event, True
                )
        else:
            GPIO.output(pin, GPIO.LOW)
        rospy.loginfo(f"Turning on pin {pin}")
        GPIO.output(pin, GPIO.HIGH)

    def handle_toggle_pin_service(self, req):
        assert isinstance(req, SetBoolRequest)
        self.toggle_pin(PIN, req.data)
        return SetBoolResponse(True, "Success")

    def handle_joy_msg(self, msg):
        assert isinstance(msg, Joy)
        if msg.buttons[JOY_BUTTON] == 1:
            self.toggle_pin(PIN, True)
        else:
            self.toggle_pin(PIN, False)

    def handle_timer_event(self, event):
        self.toggle_pin(PIN, False)


if __name__ == "__main__":
    rospy.init_node("toogle_gpio")
    toggle = ToggleGPIO()
    print(f"Ready toggle GPIO service on pin: {PIN}.")
    rospy.spin()
