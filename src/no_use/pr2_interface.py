#!/usr/bin/env python

#XXX Clean up place
#XXX fix for side grasps
# combine controller packages!  fix launch file

import numpy as np

import roslib; 
roslib.load_manifest('pr2_pick_and_place_demos')
roslib.load_manifest('ar_track_alvar')
import std_msgs.msg

from pr2_controllers_msgs.msg import JointTrajectoryAction,\
    JointTrajectoryGoal, JointTrajectoryControllerState, SingleJointPositionAction,\
    SingleJointPositionGoal, PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Twist
from ar_track_alvar.msg import AlvarMarkers
import actionlib
import rospy
import tf
from tf.transformations import euler_from_quaternion
import pdb
from Base import Base
from pr2_pick_and_place_demos.pick_and_place_manager import *
from object_manipulator.convert_functions import *
#from arm_controller import Arm
from new_papm import pick_and_place as Place

class torso:
  def __init__(self):
    self.client = actionlib.SimpleActionClient('torso_controller/position_joint_action',    
                                        SingleJointPositionAction)
    self.client.wait_for_server()
  def move_up(self):
    self.client.send_goal(SingleJointPositionGoal(position=0.1))
    #self.client.wait_for_result()
  def move_down(self):
    self.client.send_goal(SingleJointPositionGoal(position=0.0))
    #self.client.wait_for_result()

class trackObject:
  def __init__(self,id=0):
    self.pose = None
    self.pose_time = 0
    self.id = id
    self.subscriber = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback)
  def callback(self, data):
    try:
      self.data = data.markers[self.id]
      marker = data.markers[self.id]
      self.pose_time = (marker.header.stamp.secs, marker.header.stamp.nsecs)
      self.pose = marker.pose.pose
    except Exception as info:
      pass
  def get_point(self):
    data = self.data
    pose = data.pose.pose.position
    frame_id = data.header.frame_id
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = pose.x
    point.point.y = pose.y
    point.point.z = pose.z
    return point

  def get_pose(self):
    data = self.data
    pose = data.pose
    pose.header = data.header
    return pose

class Head:
  def __init__(self):
    self.client = actionlib.SimpleActionClient('head_traj_controller/point_head_action',
    PointHeadAction)
    self.client.wait_for_server()
  def lookAt (self, pose):
    goal = PointHeadGoal()
    point = PointStamped()
    point.header.frame_id = 'torso_lift_link'
    point.point.x = pose.x
    point.point.y = pose.y
    point.point.z = pose.z
    goal.target = point

    goal.pointing_frame = "head_mount_kinect_rgb_link"
    goal.pointing_frame = "head_mount_link"
    goal.min_duration = rospy.Duration(0.5)
    goal.max_velocity = 1.0

    self.client.send_goal(goal)

class PickAndPlace():

  def __init__(self, whicharm='l'):
    rospy.loginfo("initializing pick and place manager")
    #self.papm = PickAndPlaceManager()
    rospy.loginfo("finished initializing pick and place manager")
    self.table_height = self.papm.table_height 
    self.papm.move_arm_to_side(0)  #right arm
    self.papm.move_arm_to_side(1)  #left arm

    if whicharm=='l':
      self.whicharm = 1
    else:
      self.whicharm = 0

  #pick up the nearest object to PointStamped target_point with whicharm 
  #(0=right, 1=left)
  def pick_up_object_near_point(self, target_point, whicharm=1):
    if whicharm is None:
      whicharm = self.whicharm
    """
    rospy.loginfo("moving the arms to the side")
    self.papm.move_arm_to_side(0)  #right arm
    self.papm.move_arm_to_side(1)  #left arm
    """

    rospy.loginfo("pointing the head at the target point")
    self.papm.point_head(get_xyz(target_point.point),
                         target_point.header.frame_id)

    rospy.loginfo("detecting the table and objects")
    self.papm.call_tabletop_detection(update_table = 0, update_place_rectangle = 0, 
                         clear_attached_objects = 1)     

    rospy.loginfo("picking up the nearest object to the target point")
    success = self.papm.pick_up_object_near_point(target_point, 
                                                  whicharm)
    self.table_height = self.papm.table_height
    
    if success:
        rospy.loginfo("pick-up was successful!  Moving arm to side")
        #self.papm.move_arm_to_side(whicharm)
    else:
        rospy.loginfo("pick-up failed.")

    return success


  #place the object held in whicharm (0=right, 1=left) down in the 
  #place rectangle defined by place_rect_dims (x,y) 
  #and place_rect_center (PoseStamped)
  def place_object(self, place_rect_dims, place_rect_center, whicharm=None):
    if whicharm is None:
      whicharm= self.whicharm
    self.papm.set_place_area(place_rect_center, place_rect_dims)
    
    rospy.loginfo("putting down the object in the %s gripper"\
                  %self.papm.arm_dict[whicharm])
    success = self.papm.put_down_object(whicharm, 
                  max_place_tries = 5,
                  use_place_override = 1)

    if success:
        rospy.loginfo("place returned success")
    else:
        rospy.loginfo("place returned failure")

    return success

class Awesomeness:
  def __init__(self, pick_id=0, place_id=1):
    self.pick_object = trackObject(pick_id)
    self.place_object = trackObject(place_id)

    #self.pap = PickAndPlace()
    self.pick_success = True
    #self.arm = Arm('l_arm')

    self.table_point = PointStamped()
    self.table_point.header.frame_id = 0
    self.table_point.point.x = 0.475179451085
    self.table_point.point.y = 0.283060568718
    self.table_point.point.z = -0.407675985492
    self.placer = Place()
    self.head = Head()
    self.base = Base()

  def run(self):
    self.pick_up()
    self.place()
  
  def pick_up(self):
    self.pick_success = False
    self.pick_success = self.placer.pick_up()
    return self.pick_success

  def papmpick_up(self):
    self.pick_success = False
    point = self.pick_object.get_point()
    #self.pick_success = self.pap.pick_up_object_near_point(self.table_point)  
    self.pick_success = self.pap.pick_up_object_near_point(point)  
  
  def place(self):
      self.place_success = False
      return self.placer.place_can()
     
  def papmplace(self):
    if self.pick_success:
      #square of size 10 cm by 10 cm
      place_rect_dims = [.1, .1]                              
      place_rect_center = self.place_object.get_pose()
      self.pap.place_object(place_rect_dims, place_rect_center)
  
  def drop(self):
     self.pap.papm.open_gripper(1)  # 1 is for left arm

  def rotate(self, amount):
    self.base.rotateBase(amount)



if __name__ == "__main__":
  rospy.init_node('simple_pick_and_place_example')
  awesome = Awesomeness()
  times = dict(zip(['pickup', 'place'], [[], []]))
  for i in range(5):
    
    #raw_input("pick up object?")
    start = rospy.get_rostime().to_sec()
    result = awesome.pick_up()
    while not result:
        result = awesome.pick_up()
        continue
    end = rospy.get_rostime().to_sec()
    times['pickup'].append(int(end - start)+1) # ceiling round off
    
    #raw_input("place object?")
    rospy.sleep(1)  # wait for robot to see AR tag
    start = rospy.get_rostime().to_sec()
    result = awesome.place()
    while not result:
        result = awesome.place()
    end = rospy.get_rostime().to_sec()
    times['place'].append(int(end - start)+1) # ceiling round off
    
    
    with open("time_results.csv", "wb") as f:
      for key in times:
          f.write( "%s, %s\n" % (key, ", ".join(repr(e) for e in times[key])))

