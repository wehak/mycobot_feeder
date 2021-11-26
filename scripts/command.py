#!/usr/bin/env python2
import time
from math import pi, cos, sin, atan2

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class Command():

   def __init__(self):
      pose_bu = None
      posf_bf = None

      rospy.init_node('command', anonymous=False)
      rospy.Subscriber('pose/bu', PoseStamped, self.get_bu_pose)
      rospy.Subscriber('pose/bf', PoseStamped, self.get_bf_pose)

      self.init_moveit_mycobot()
      self.menu()

   def init_moveit_mycobot(self):
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.move_group = moveit_commander.MoveGroupCommander("arm_group")

      self.display_trajectory_publisher = rospy.Publisher(
         "/move_group/display_planned_path",
         moveit_msgs.msg.DisplayTrajectory,
         queue_size=20,
      )

   # reads a "geometry_msgs/PoseStamped Message" and return a dict with numpy arrays 
   # with position vector and orientation quaternion
   def read_pose_msg(self, data):
      pose = {}
      pose["position"] = np.array([
         data.pose.position.x,
         data.pose.position.y,
         data.pose.position.z,
      ])

      pose["orientation"] = np.array([
         data.pose.orientation.x,
         data.pose.orientation.y,
         data.pose.orientation.z,
         data.pose.orientation.w,
      ])
   
      return pose

   # callback function for topic subscriber
   def get_bu_pose(self,data):
      self.pose_bu = self.read_pose_msg(data)

   # callback function for topic subscriber
   def get_bf_pose(self,data):
      self.pose_bf = self.read_pose_msg(data)
   
   def menu(self):
      print("-------------------------------------")
      print("\tPlease select:")
      print("\t1. Reset MyCobot position")
      print("\t2. Send MyCobot to food position")
      print("\t3. Send MyCobot to user position")
      print("\t4. Exit")
      print("-------------------------------------")

      choice = -1
      while choice < 1 or choice > 4:
         try:
            choice = int(raw_input())
         except:
            print("Must be a integer between 1 and 41")
      
      if choice == 1:
         self.reset_mycobot()
      elif choice == 2:
         self.goto_food()
      elif choice == 3:
         self.goto_user()
      elif choice == 4:
         exit()
      else:
         print("Error \"{}\" choice" .format(choice))

   def reset_mycobot(self):
      joint_goal = self.move_group.get_current_joint_values()
      joint_goal[0] = 0
      joint_goal[1] = -pi/4
      joint_goal[2] = pi/2
      joint_goal[3] = -pi/4
      joint_goal[4] = pi
      joint_goal[5] = 0

      self.move_group.go(joint_goal, wait=True)
      self.move_group.stop()

      time.sleep(1)
      self.menu()

   def goto_food(self):
      
      print("Sending end effector to food at: \nx={} \ny={} \nz={} " .format(
         self.pose_bf["position"][0],
         self.pose_bf["position"][1],
         self.pose_bf["position"][2]# + 0.05
      ))

      pose_goal = Pose()
      pose_goal.orientation.w = -1
      pose_goal.position.x = self.pose_bf["position"][0]
      pose_goal.position.y = self.pose_bf["position"][1]
      pose_goal.position.z = self.pose_bf["position"][2] + 0.05


      theta = atan2(
         self.pose_bf["position"][0],
         self.pose_bf["position"][1]
      )
      new_r = R.from_dcm([
         [cos(theta),   -sin(theta),   0],
         [sin(theta),   cos(theta),    0],
         [0,            0,             -1],
      ]).as_quat()

      pose_goal.orientation.x = new_r[0]
      pose_goal.orientation.y = new_r[1]
      pose_goal.orientation.z = new_r[2]
      pose_goal.orientation.w = new_r[3]

      self.move_group.set_pose_target(pose_goal)
      plan = self.move_group.go(wait=True)
      self.move_group.stop()
      self.move_group.clear_pose_targets()

      time.sleep(1)
      self.menu()

   def goto_user(self):
      
      print("\nSending end effector to user at: \nx={} \ny={} \nz={} " .format(
         self.pose_bu["position"][0],
         self.pose_bu["position"][1],
         self.pose_bu["position"][2]# + 0.05
      ))

      pose_goal = Pose()
      pose_goal.orientation.w = -1
      pose_goal.position.x = self.pose_bu["position"][0]
      pose_goal.position.y = self.pose_bu["position"][1]
      pose_goal.position.z = self.pose_bu["position"][2]

      # flip the orientation 180 degrees relative to user
      # old_r = R.from_quat([
      #    self.pose_bu["orientation"][0],
      #    self.pose_bu["orientation"][1],
      #    self.pose_bu["orientation"][2],
      #    self.pose_bu["orientation"][3],
      # ]).as_dcm()

      # new_r = R.from_dcm([
      #    [],
      #    [],
      #    [],
      # ]).as_quat()
      # new_r = R.from_dcm(np.linalg.inv(old_r)).as_quat()

      # new_r = R.from_dcm([
      #    [-1, 0, 0],
      #    [0, 1, 0],
      #    [0, 0, -1]
      # ]).as_quat()


      theta = atan2(
         self.pose_bu["position"][0],
         self.pose_bu["position"][1]
      )
      new_r = R.from_dcm([
         [cos(theta),   -sin(theta),   0],
         [sin(theta),   cos(theta),    0],
         [0,            0,             -1],
      ]).as_quat()

      pose_goal.orientation.x = new_r[0]
      pose_goal.orientation.y = new_r[1]
      pose_goal.orientation.z = new_r[2]
      pose_goal.orientation.w = new_r[3]

      self.move_group.set_pose_target(pose_goal)
      plan = self.move_group.go(wait=True)
      self.move_group.stop()
      self.move_group.clear_pose_targets()


      self.menu()



if __name__ == '__main__':
   cmd = Command()