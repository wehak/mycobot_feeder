#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pymycobot.mycobot import MyCobot

mc = None
rate = None

def callback(data):
    rospy.loginfo(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    mc.send_radians(data_list, 80)
    rate.sleep()


def mycobot_controller():
    global mc, rate

    # initiate variables pub joint state
    joint_state_send = JointState()
    joint_state_send.header = Header()

    joint_state_send.name = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]
    joint_state_send.velocity = [0]
    joint_state_send.effort = []

    # initiate mycobot
    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    rospy.loginfo("MyCobot connecting to port={} with baud={}" .format(port, baud))
    mc = MyCobot(port, baud)

    # start the node
    rospy.init_node("mycobot_controller", anonymous=True)
    rate = rospy.Rate(30) # Hz

    # start the subscriber
    rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, callback)

    # start the publisher
    pub = rospy.Publisher("joint_state_status", JointState, queue_size=10)

    # publish while ros is running
    while not rospy.is_shutdown():
        angles = mc.get_angles()
        joint_state_send.header.stamp = rospy.Time.now()
        joint_state_send.position = angles
        rospy.loginfo(joint_state_send)
        pub.publish(joint_state_send)
        rate.sleep()

if __name__ == "__main__":
    mycobot_controller()
