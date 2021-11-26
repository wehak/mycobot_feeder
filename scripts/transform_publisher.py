#!/usr/bin/env python2

import collections

import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Header
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class PoseSubscriber():
    def __init__(self):
        self.T_ct = np.zeros((4, 4))
        self.T_cf = np.zeros((4, 4))
        self.T_cu = np.zeros((4, 4))

        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.get_pose_transform)

    def get_T(self, x):
        # get the rotation of object x
        R_matrix = R.from_quat([
            x.transform.rotation.x,
            x.transform.rotation.y,
            x.transform.rotation.z,
            x.transform.rotation.w,
        ]).as_dcm()

        # translation vector
        p = np.array([
            x.transform.translation.x,
            x.transform.translation.y,
            x.transform.translation.z
        ])

        # create the homogeneous transformation matrix from R and translation vector
        T = np.zeros((4, 4))
        T[:3, :3] = R_matrix
        T[:3, 3] = p.T
        T[3, 3] = 1
        return T

    def get_pose_transform(self, data):

        self.T_ct = np.zeros((4, 4))
        self.T_cf = np.zeros((4, 4))
        self.T_cu = np.zeros((4, 4))
        # print(rospy.get_caller_id())
        for x in data.transforms:

            # calculate position of robot (t)ag in relation to camera frame
            if x.fiducial_id == 3:
                self.T_ct = self.get_T(x)
            
            # calculate position of (f)ood in relation to camera frame
            elif x.fiducial_id == 1:
                self.T_cf = self.get_T(x)
            
            # calculate position of (u)ser in relation to camera frame
            elif x.fiducial_id == 2:
                self.T_cu = self.get_T(x)

            # ignore all other  tags
            else:
                continue
        
if __name__ == "__main__":

    # formulate a pose message to publish
    def get_Pose_msg(T):
        """
        creates a:
        geometry_msgs/Pose Message
        --------------------------
        from:
        geometry_msgs/Point position
        geometry_msgs/Quaternion orientation
        """

        # extract rotation matrix and translation vector from T
        R_matrix = T[:3, :3]
        p = T[:3, 3]

        # convert rotation matrix to quaternion and formulate message
        r = R.from_dcm(R_matrix).as_quat()

        quat = Quaternion()
        quat.x = r[0]
        quat.y = r[1]
        quat.z = r[2]
        quat.w = r[3]

        # forumate translation as a Point mesg
        translation = Point()
        translation.x = p[0]
        translation.y = p[1]
        translation.z = p[2]

        # create msg
        msg = Pose()
        msg.position = translation
        msg.orientation = quat

        return msg
    
    # initialize the node and create publishers
    rospy.init_node("transform_publisher", anonymous=False)
    pub_bu = rospy.Publisher("pose/bu", PoseStamped, queue_size=30)
    pub_bf = rospy.Publisher("pose/bf", PoseStamped, queue_size=30)

    # rate at which the node will publish (set to match camera FPS)
    rate = rospy.Rate(30) # Hz

    # initate subscriber object
    pose = PoseSubscriber()

    # define position of robot (b)ase in relation to robot (t)ag frame
    T_bt = np.array([
        [1, 0, 0, 0.0],
        [0, 0, -1, -0.045],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    # set up calculation of rolling average
    T_bf_stack = collections.deque()
    T_bu_stack = collections.deque()
    rolling_avg_n = 10 # samples in the rolling avg.

    # main loop for publisher
    while not rospy.is_shutdown():

        """position of (f)ood in relation to robot (b)ase frame """
        # if both matrices are detected in last frame
        if np.any(pose.T_ct) and np.any(pose.T_cf):

            # calculate the position of the (f)ood in relation to the robot (b)ase
            T_bf_stack.appendleft(T_bt.dot( inv(pose.T_ct) ).dot( pose.T_cf ))

            # calculate the rolling average from the stack of len() equal "rolling_avg_n"
            if len(T_bf_stack) > rolling_avg_n:
                T_bf_stack.pop()
            T_bf_avg = np.mean( T_bf_stack, axis=0)
            
            # create msg
            msg = PoseStamped()
            msg.pose = get_Pose_msg(T_bf_avg)
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            pub_bf.publish(msg)
        
        # if tags are hidden and no pose was calculated, remove the last entry from rolling average
        else:
            if len(T_bf_stack) > 0:
                T_bf_stack.pop()

        """position of (u)ser in relation to robot (b)ase frame """
        # if both matrices are detected in last frame
        if np.any(pose.T_ct) and np.any(pose.T_cu):

            # calculate the position of the (u)ser in relation to the robot (b)ase
            T_bu_stack.appendleft(T_bt.dot( inv(pose.T_ct) ).dot( pose.T_cu ))

            # calculate the rolling average from the stack of len() equal "rolling_avg_n"
            if len(T_bu_stack) > rolling_avg_n:
                T_bu_stack.pop()
            T_bu_avg = np.mean( T_bu_stack, axis=0)

            # create msg
            msg = PoseStamped()
            msg.pose = get_Pose_msg(T_bu_avg)
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            pub_bu.publish(msg)
        
        # if tags are hidden and no pose was calculated, remove the last entry from rolling average
        else:
            if len(T_bu_stack) > 0:
                T_bu_stack.pop()

        rate.sleep() # keep up with the camera FPS
