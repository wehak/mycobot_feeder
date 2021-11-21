#!/usr/bin/env python2

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""

import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

# set the update rate of publisher and subscriber
rate = rospy.Rate(10) # Hz
mc = None

def callback(data):
    # global mc, rate

    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    rospy.loginfo(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    mc.send_radians(data_list, 80)
    # time.sleep(0.5)
    rate.sleep()


def mycobot_controller():
    global mc

    # initiate mycobot
    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    rospy.loginfo(f"MyCobot connecting to port={port} with baud={baud}")
    mc = MyCobot(port, baud)

    # start the node
    rospy.init_node("mycobot_controller", anonymous=True)

    # start the subscriber
    rospy.Subscriber("joint_state_commands", JointState, callback)

    # start the publisher
    pub = rospy.Publisher("joint_state_status", JointState, queue_size=10)
    while not rospy.is_shutdown():
        states = mc.get_angles()
        rospy.loginfo(states)
        pub.publish(states)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    # print("spin ...")
    # rospy.spin()


if __name__ == "__main__":
    mycobot_controller()