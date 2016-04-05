#!/usr/bin/env python
#Author: Ariel Anders

import numpy as np
import rospy
import roslib; 
roslib.load_manifest("pr2_awesomeness")
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction,\
        Pr2GripperCommandGoal, Pr2GripperCommand
from pr2_gripper_sensor_msgs.msg import  \
        PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal
from sensor_msgs.msg import JointState
from object_manipulator.convert_functions import *
"""
This file has gripper functions. General- open close and gripper event
detection
"""
class GripperDetector:
    def __init__(self, blocking=False):
        self.gripper_event_detector_action_client = {}
        self.client = {}
        self.arms = ['l','r']
        for arm in self.arms:
            self.client[arm] =  actionlib.SimpleActionClient(\
                    arm+'_gripper_sensor_controller/event_detector',\
                    PR2GripperEventDetectorAction)
        if blocking:
            for arm in self.arms:
                self.wait_for_action_server(self.client[arm],
                         arm+'_gripper_sensor_controller/event_detector')
    
    
    ##start up gripper event detector to detect when an object hits the table 
    #or when someone is trying to take an object from the robot
    def start_gripper_event_detector(self, whicharm, blocking = True, timeout = 5):
        goal = PR2GripperEventDetectorGoal()
        
        #use either slip or acceleration as a contact condition
        goal.command.trigger_conditions = \
                goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC 
                #goal.command.FINGER_SIDE_IMPACT_OR_ACC 
        
        #contact acceleration used to trigger 
        goal.command.acceleration_trigger_magnitude = 3.25 
        #contact slip used to trigger    
        goal.command.slip_trigger_magnitude = 0.008        

        rospy.loginfo("starting gripper event detector")
        self.client[whicharm].send_goal(goal)

        #if blocking is requested, wait until the action returns
        if blocking:
             self.client[whicharm].wait_for_result(rospy.Duration(timeout))
             state = self.client[whicharm].get_state()
             print state
    def event_detected(self, whicharm):
        state = self.client[whicharm].get_state()
        return state
   
class GripperController:
    def __init__(self, event_detector = False):
        self.arms= ['r','l']
        self.client = {}
        for arm in self.arms:
            self.client[arm] = actionlib.SimpleActionClient(\
              arm+"_gripper_controller/gripper_action",Pr2GripperCommandAction)
        # for gripper listener:
        rospy.loginfo("subscribed to gripper controller")
        self.gripper_positions = {}
        self.gripper_efforts = {}
        self.sub = rospy.Subscriber("joint_states", JointState, self.callback)
        self.threshold = .01 #XXX CHANGE ME

        if event_detector:
            self.event_detector = GripperDetector()
        else:
            self.event_detector = None
    

    def command(self, whicharm, position, max_effort=-1.0, blocking=True):
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.client[whicharm].send_goal(goal)
        if blocking:
            self.client[whicharm].wait_for_result()
            return self.client[whicharm].get_state()
        

    def open(self, whicharm):
        self.command(whicharm, .1)

    def close(self, whicharm):
        self.command(whicharm, 0, 100)

    def callback(self, data):
        joint_states = dict(zip(data.name, data.position))
        effort_states = dict(zip(data.name, data.effort))
        for arm in self.arms:
            self.gripper_positions[arm] = joint_states[arm +"_gripper_joint"]
            self.gripper_efforts[arm] = joint_states[arm +"_gripper_joint"]
    
    def get_gripper_positions(self):
        return self.gripper_positions, self.gripper_efforts

    def is_grasping(self, threshold=None):
        result = {}
        # also test if effort applied?
        if threshold==None:
            threshold = self.threshold
        for arm in self.arms:
            result[arm] = self.gripper_positions[arm] > threshold
        return result 

    def start_gripper_event_detector(self, whicharm, blocking = True, timeout = 5):
        if self.event_detector == None:
            rospy.loginfo("Gripper sensor action detector not loaded."
            "Returning True")
            return True
        else:
            self.event_detector.start_gripper_event_detector(whicharm,\
                    blocking = True, timeout = 5)

    def event_detected(self, whicharm):
        if self.event_detector == None:
            rospy.loginfo("Gripper sensor action detector not loaded."
            "Returning True")
            return True
        else:
            self.event_detector.event_detected(whicharm)

if __name__=="__main__":
    rospy.init_node("simplegrippertest")
    
    cg =GripperController()
    #raw_input("open gripper?")
    #cg.open("r")
    raw_input("close gripper?")
    cg.close("r")
    raw_input("check if grasping?")
    print cg.is_grasping()
    raw_input("print current gripper location?")
    print cg.get_gripper_positions()
    raw_input("check if event dected?")
    print cg.start_gripper_event_detector("r")

