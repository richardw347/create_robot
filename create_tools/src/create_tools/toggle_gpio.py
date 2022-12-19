#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO


PIN = 18
JOY_BUTTON = 5


def toggle_pin(pin, state):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)
    if state:
        GPIO.output(pin, GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.LOW)
    rospy.loginfo(f"Turning on pin {pin}")
    GPIO.output(pin, GPIO.HIGH)


def handle_toggle_pin_service(req):
    assert isinstance(req, SetBoolRequest)
    toggle_pin(PIN, req.data)
    return SetBoolResponse(True, "Success")


def handle_joy_msg(msg):
    assert isinstance(msg, Joy)
    if msg.buttons[JOY_BUTTON] == 1:
        toggle_pin(PIN, True)
    else:
        toggle_pin(PIN, False)


if __name__ == "__main__":
    rospy.init_node("toogle_gpio")
    joy_sub = rospy.Subscriber("joy", Joy, handle_joy_msg)
    s = rospy.Service("toggle_gpio", SetBool, handle_toggle_pin_service)
    print(f"Ready toggle GPIO service on pin: {PIN}.")
    rospy.spin()
