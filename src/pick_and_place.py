#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_awesomeness")
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion
import actionlib
from ar_track_alvar.msg import AlvarMarkers
import numpy as np
from awesome_arm_controller import ArmController
from pick_up_controller import ArghGrasp
from tf_transformer import TfTransformer
from argparse import ArgumentParser


class pick_and_place():
    def __init__(self, pick_id=0, place_id=1):
        
        self.pick_pose = [-999]*7
        self.pick_data = False
        self.place_pose = [-999]*7
        self.place_data = False
        
        self.subscriber = rospy.Subscriber('ar_pose_marker', AlvarMarkers, \
                self.ar_callback)
        self.ctrl = ArmController(event_detector=True)
        self.tf_transformer = TfTransformer(self.ctrl.tf_listener)
        self.joint_move_arm_to_side('r', blocking=False)
        self.joint_move_arm_to_side('l')
        self.default_frame = "base_link"
        self.grasper = ArghGrasp()
    """
    ### experimental code
    def pick_up(self, whicharm='l', top_grasp = False):
        rospy.loginfo("pre pickup pose")
        self.open_gripper(whicharm)

        first =    [1.042181865195266, 0.05674577918307219, 1.7141703410128142, -1.2368941648991192, 44.01523148337816, -1.3485333030083617, 58.22465995394929]
        self.ctrl.joint_movearm(whicharm, first)
        start = [0.6169387251649058, 0.13840014757000843, 0.7266251135486242, 0.7131394766246717, -0.7008370665851971, 0.008851073879117963, 0.013459252242671303]

        end = [0.591262509439721, -0.19429620128938668, 0.748909628486445, 0.7368007601058577, -0.6756313110616471, -0.01694214819160356, -0.01897195391107014]

        
        self.move_cartesian_step(whicharm, start)
        self.move_cartesian_step(whicharm, end, blocking=False)

        self.ctrl.gripper.start_gripper_event_detector(\
            whicharm, blocking=True,timeout = 5)
        self.ctrl.cart_freeze_arm(whicharm)
        self.close_gripper(whicharm)
        
        curr = self.ctrl.get_cartesian_pose()[whicharm]
        curr[2] += 0.05
        self.move_cartesian_step(whicharm,curr)
        self.constrained_move_arm_to_side(whicharm)
        return True
    """        


    def pick_up(self, whicharm='l'):
        if self.pick_data:
            self.open_gripper(whicharm) 
            if not self.grasper.pick_up(whicharm,5, self.pick_pose):
                return False
            # move arm to side
            rospy.loginfo("moving arm to side")
            self.constrained_move_arm_to_side(whicharm)
            return self.ctrl.gripper.is_grasping()[whicharm]
        else:
            return False
            
    def reset_ar(self):
        self.pick_data = False
        self.place_data = False
    
    def place(self, whicharm="l", top_grasp = True, use_offset = False):
        if self.place_data:
            # get place offset from cooler
            if use_offset:
                trans, _ = self.tf_transformer.get_transform(\
                'base_link', 'ar_marker_1')
                if not trans:
                    return False
            else:
                print "not finding place pose"
                trans = self.place_pose[:3]     
            curr_pose = self.ctrl.get_cartesian_pose()[whicharm]
            rot = curr_pose[3:]
            #_,rot = self.ctrl.get_cartesian_pose()[whicharm]

            place_pose = trans + rot
            over_pose, under_pose = list(place_pose), list(place_pose)
            over_pose[2] += .2
            under_pose[2] -= .2
            self.reset_ar()
            print "%s\n%s\n%s"%(place_pose, over_pose, under_pose)
            rospy.loginfo("moving arm over place pose")
            result = self.move_cartesian_step(whicharm,over_pose) 
            
            if result:

                rospy.loginfo("guarded move arm to place pose")
                result = self.guarded_cartesian_move(whicharm, under_pose)
                
                if result:
                    rospy.loginfo("placing can")
                    self.open_gripper(whicharm)
            
                    rospy.loginfo("moving arm up over place pose")
                    self.move_cartesian_step(whicharm, over_pose)
                else:
                    rospy.loginfo("failed doing guarded move")    
            else:
                rospy.loginfo("failed moving arm over")

            """
            rospy.loginfo("moving arm up over place pose")
            tmp = False
            for i in range (5):
                if tmp:
                    break
                tmp = self.move_cartesian_step(whicharm, over_pose)
                if not tmp:
                    rospy.loginfo("having difficulty moving arm up "
                    "over place pose")
            if not tmp:
                raw_input("can't find over pose")
            """

            self.joint_move_arm_to_side(whicharm)
            rospy.loginfo("place result succes is: %s " % result)
            return result
        else:
            rospy.loginfo("no place data")
            return False
    
    def ar_callback(self, data):
        for markers in data.markers: 
            try:
                px = markers.pose.pose.position.x
                py = markers.pose.pose.position.y
                pz = markers.pose.pose.position.z
                ox = markers.pose.pose.orientation.x
                oy = markers.pose.pose.orientation.y
                oz = markers.pose.pose.orientation.z
                ow = markers.pose.pose.orientation.w

                if markers.id == 0: # pick
                    self.pick_pose = [px,py,pz,ox,oy,oz,ow]
                    self.pick_data = True
                if markers.id == 1: # place
                    self.place_pose = [px, py, pz, ox, oy, oz, ow]
                    self.place_data = True
            except:
                pass
    
    #open the gripper 
    def open_gripper(self, whicharm):
        self.ctrl.open_gripper(whicharm)


    #close the gripper 
    def close_gripper(self, whicharm):
        self.ctrl.close_gripper(whicharm)


    def constrained_move_arm_to_side(self, whicharm):
        location={'l': [0.05, 0.65, .7], 'r': [0.05, -0.65, .7]}
        poses = self.ctrl.get_cartesian_pose()
        rot = poses[whicharm][3:]
        side_pose = location[whicharm] + rot
        result = self.move_cartesian_step(whicharm, side_pose, blocking=True)
        if not result:
            rospy.loginfo("cannot constrain joint moving to side")
            self.joint_move_arm_to_side(whicharm)

   
    def joint_move_arm_to_side(self, whicharm, blocking = 1):
        self.arm_to_side_angles = \
                {'r': [-2.02, 0.014, -1.73, -1.97, 10.91, -1.42, 14.17],
                 'l': [2.02, 0.014, 1.73, -1.97, -10.91, -1.42, 14.17]}
        self.ctrl.joint_movearm(whicharm,self.arm_to_side_angles[whicharm],\
                blocking=blocking)
 
    def guarded_cartesian_move(self, whicharm, pose, frame="base_link",\
            move_duration = 15):
        start_pose = self.ctrl.get_cartesian_pose()[whicharm][:3]
        self.move_cartesian_step(whicharm, pose, move_duration=7.0,\
                    frame_id=frame, blocking=False)
        self.ctrl.gripper.start_gripper_event_detector(\
            whicharm, blocking=True,timeout=move_duration)
        self.ctrl.cancel_goal(whicharm)
        # did arm even move?    
        curr_pose = self.ctrl.get_cartesian_pose()[whicharm][:3]
        diff = np.linalg.norm(np.array(start_pose) - np.array(curr_pose))
        not_at_start = diff > 0.1
        return not_at_start
        
    #move to a Cartesian pose goal
    def move_cartesian_step(self, whicharm, pose, frame_id="base_link",\
            move_duration=5.0, ik_timeout=5.0, blocking=True):

        result = self.ctrl.cart_movearm(whicharm, pose, frame_id, \
        move_duration, ik_timeout, blocking) 
        return result
    
   
def complete_timing_trials():
  rospy.init_node('time_trials')
  times = dict(zip(['pickup', 'place'], [[], []]))
  awesome = pick_and_place()

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
    result = awesome.place(use_offset=False)
    while not result:
        result = awesome.place()
    end = rospy.get_rostime().to_sec()
    times['place'].append(int(end - start)+1) # ceiling round off
    
    
    with open("time_results.csv", "wb") as f:
      for key in times:
          f.write( "%s, %s\n" % (key, ", ".join(repr(e) for e in times[key])))

if __name__=="__main__":
    parser = ArgumentParser(\
        description="which script should we run for pick and place demo")
    parser.add_argument("-t", "--time", help="complete time trials",\
        action="store_true", dest="time")
    parser.add_argument("-c", "--continuous", help="run continuously",\
        action="store_true", dest="continuous")
    parser.add_argument("--place", help="only do place action",\
        action="store_true")
    parser.add_argument("--pick", help="only do pick action",\
        action = "store_true")


    args= parser.parse_args()
    if args.time:
        complete_timing_trials()
    else:
        
        rospy.init_node("pick_and_place")
        pm = pick_and_place()
        while True:
            result = False
            result = args.place
            while not result:
              result = pm.pick_up()
            result = False

            result = args.pick
            while not result:
              print "hello!"
              result =  pm.place()
              if not result:
                  rospy.loginfo("place failed")
                  rospy.sleep(.5)
            if not args.continuous:
                break

