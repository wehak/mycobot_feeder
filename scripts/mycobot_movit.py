#!/usr/bin/env python2

import sys
from math import pi
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("kinematics", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_group")

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

print("\nPlanning frame:\n", move_group.get_planning_frame(), "\n")
print("\nEEf link:\n", move_group.get_end_effector_link(), "\n")
print("\nGroup names:\n", robot.get_group_names(), "\n")
# print("\nRobot state:\n", robot.get_current_state(), "\n")

# joint_goal = move_group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = -pi/4
# joint_goal[2] = pi/2
# joint_goal[3] = -pi/4
# joint_goal[4] = pi
# joint_goal[5] = 0

# move_group.go(joint_goal, wait=True)
# move_group.stop()

# time.sleep(3)

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1
pose_goal.position.x = 0.0
pose_goal.position.y = 0.2
pose_goal.position.z = 0.2

move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

print("Done")