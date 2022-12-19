#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO


DEFAULT_PIN = 18
DEFAULT_JOY_BUTTON = 1
DEFAULT_SAFETY_TIMEOUT = 2


class ToggleGPIO:
    def __init__(
        self, pin: int, joy_button: int, safety_timeout: int, invert: bool = False
    ):
        self.pin = pin
        self.joy_button = joy_button
        self.safety_timeout = safety_timeout
        self.invert = invert
        self.timer = None
        self.setup_pin()
        self.joy_sub = rospy.Subscriber("joy", Joy, self.handle_joy_msg)
        self.service = rospy.Service(
            "toggle_gpio", SetBool, self.handle_toggle_pin_service
        )

    def setup_pin(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

    def set_pin(self, pin, state):

        if self.invert:
            state = not state

        if state:
            GPIO.output(pin, GPIO.HIGH)
            if self.timer is not None:
                self.timer = rospy.Timer(
                    rospy.Duration(self.safety_timeout), self.handle_timer_event, True
                )
        else:
            GPIO.output(pin, GPIO.LOW)

    def handle_toggle_pin_service(self, req):
        assert isinstance(req, SetBoolRequest)
        self.set_pin(self.pin, req.data)
        return SetBoolResponse(True, "Success")

    def handle_joy_msg(self, msg):
        assert isinstance(msg, Joy)
        if msg.buttons[self.joy_button] == 1:
            self.set_pin(self.pin, True)
        else:
            self.set_pin(self.pin, False)

    def handle_timer_event(self, event):
        self.set_pin(self.pin, False)


if __name__ == "__main__":
    rospy.init_node("toogle_gpio")
    pin = rospy.get_param("~pin", DEFAULT_PIN)
    joy_button = rospy.get_param("~joy_button", DEFAULT_JOY_BUTTON)
    safety_timeout = rospy.get_param("~safety_timeout", DEFAULT_SAFETY_TIMEOUT)
    invert = rospy.get_param("~invert", True)
    toggle = ToggleGPIO(pin, joy_button, safety_timeout, invert)
    print(f"Ready toggle GPIO service on pin: {pin}.")
    rospy.spin()
    GPIO.cleanup()
