import roslib
roslib.load_manifest("pr2_awesomeness")
import rospy
#from object_manipulator.convert_functions import *
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion
import actionlib
from ar_track_alvar.msg import AlvarMarkers

from awesome_arm_controller import ArmController
from pick_up_controller import ArghGrasp
from controller_manager import ControllerManager
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
        """
        self.pick_pose = [0.475, .283, 0.3, 0,0,0,0]
        self.place_pose = [0.475, .283, 0.3, 0,0,0,0]
        self.place_data = True
        self.pick_data = True
        """
        self.ctrl = ArmController(event_detector=True)
        self.tf_transformer = TfTransformer(self.ctrl.tf_listener)
        self.new_ctrl = ControllerManager('l')
        self.new_ctrl.start_joint_controllers()
        self.joint_move_arm_to_side('r', blocking=False)
        self.joint_move_arm_to_side('l')
        self.default_frame = "base_link"
        print "hello!"
        self.grasper = ArghGrasp()
        print "moved to pickup"

    """
    def pick_up(self, whicharm='l', top_grasp = False):
        if self.pick_data:

            x,y,z,_,_,_,_ = self.pick_pose
            if top_grasp:
                rot = [-.5, .5, .5, .5]
                over_pos = [x,y,z+.4] + rot
                under_pos = [x,y,z-.1] + rot
            else:
                rot = [1, 0,0,0]
                over_pos = [x-.1,y,z+.1] + rot
                front_pos = [x-.3,y,z+.1] + rot
                under_pos = [x+.1,y,z+.1] + rot

            self.open_gripper(whicharm) 
            # move arm above can
            #raw_input("move arm")
            rospy.loginfo("moving arm above can")
            self.move_cartesian_step(whicharm, over_pos)
            if not top_grasp:
                self.move_cartesian_step(whicharm,front_pos)
            #raw_input("guarded move?")
            rospy.loginfo("moving arm to can")
            self.guarded_cartesian_move(whicharm, under_pos)
            raw_input("grasp")
            rospy.loginfo( "grasping can")
            self.close_gripper(whicharm)
            #raw_input("return?")
            # lift can straight up
            rospy.loginfo( "lifting can")
            self.move_cartesian_step(whicharm, over_pos)
            
            # move arm to side
            self.constrained_move_arm_to_side(whicharm)
            return True
        else:
            return False
            
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
    
    def place(self, whicharm="l", top_grasp = True, use_offset = True):
        if self.place_data:
            # get place offset from cooler
            if use_offset:
                trans, _ = self.tf_transformer.get_transform(\
                'base_link', 'ar_marker_1')
                if not trans:
                    return False
            else:
                trans = self.place_pose[:3]        
            
            _,rot = self.new_ctrl.return_cartesian_pose()
            
            place_pose = trans + rot
            over_pose, under_pose = list(place_pose), list(place_pose)
            over_pose[2] += .2
            under_pose[2] -= .2
            self.reset_ar()
            print "%s\n%s\n%s"%(place_pose, over_pose, under_pose)
            rospy.loginfo("moving arm over place pose")
            result = self.move_cartesian_step(whicharm,over_pose) 
            
            if not result:
                rospy.loginfo("failed moving arm over")
                return result

            rospy.loginfo("guarded move arm to place pose")
            result = self.guarded_cartesian_move(whicharm, under_pose)

            if not result:
                rospy.loginfo("failed doing guarded move")
                return result

            rospy.loginfo("placing can")
            self.open_gripper(whicharm)


            rospy.loginfo("moving arm up over place pose")
            tmp = False
            for i in range (5):
                if tmp:
                    break
                tmp = self.move_cartesian_step(whicharm, over_pose)
                if not tmp:
                    rospy.loginfo("having difficulty moving arm up over place pose")
            if not tmp:
                raw_input("can't find over pose")

            self.joint_move_arm_to_side(whicharm)
            rospy.loginfo("place result succes is: %s " % result)
            return result
        else:
            rospy.loginfo("no place data")
            return False

    """
        if self.place_data:
            #todo test side grasp... and automatically detect if side or top 
            x,y,z,_,_,_,_ = self.place_pose
            trans, rot = self.new_ctrl.return_cartesian_pose()
            if top_grasp:
                #rot = [-.5, .5, .5, .5]
                over_pos = [x,y,z+.3] + rot
                under_pos = [x,y,z-.2] + rot
                place_pos = [x,y,z] + rot
            else:
                #rot = [1, 0,0,0]
                over_pos = [x,y,z+.2] + rot
                under_pos = [x,y,z-.2] + rot
                place_pos = [x,y,z] + rot
                side_pos = [x-.2, y, z] + rot
            self.reset_ar()

            rospy.loginfo("moving arm above place pose")
            result = self.move_cartesian_step(whicharm, over_pos)
            if not result: 
                  rospy.loginfo("failure to move arm over cartesian pose %s" %\
                  over_pos)
                  return False
            rospy.loginfo("guarded moving arm to place pose")
            result = self.guarded_cartesian_move(whicharm, under_pos)

            if  result:
                rospy.loginfo( "placing can")
                self.open_gripper(whicharm)
            else:
                rospy.loginfo("failed to move arm down to place pose %s " % place_pos)

            rospy.loginfo( "lifting arm away from place pose")
            self.move_cartesian_step(whicharm, over_pos)
            self.joint_move_arm_to_side(whicharm)
            rospy.loginfo("place result succes is: %s " % result)
            return result
        else:
            rospy.loginfo("no place data")
            return False
    """
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
        #self.new_ctrl.start_joint_controllers()
        self.ctrl.joint_movearm(whicharm,self.arm_to_side_angles[whicharm],\
                blocking=blocking)
 
    def guarded_cartesian_move(self, whicharm, pose, frame="base_link",\
            move_duration = 15):
        start_pose = self.new_ctrl.get_current_wrist_pose_stamped()
        start = rospy.Time.now().to_sec()
        self.move_cartesian_step(whicharm, pose, move_duration=7.0,\
                    frame_id=frame, blocking=False)
        self.ctrl.gripper.start_gripper_event_detector(whicharm, blocking=True,timeout=move_duration)
        self.ctrl.cancel_goal(whicharm)
        # did arm even move?
        at_start = self.new_ctrl.check_cartesian_done(start_pose,.15,.0)
        # assume if arm moved more than .15 m then we got close enough to
        #goal to place otherwise return false since we got stuck hovering 
        return not at_start

   

    def old_guarded_cartesian_move(self, whicharm, pose, frame="base_link",\
            move_duration = 10):
        start_pose = self.new_ctrl.get_current_wrist_pose_stamped()
        start = rospy.Time.now().to_sec()
        self.ctrl.gripper.start_gripper_event_detector(whicharm, blocking=False,timeout=1)
        continue_move = True
        self.move_cartesian_step(whicharm, pose, move_duration=10.0,\
                    frame_id=frame, blocking=False)
        while continue_move:
            event_detected = self.ctrl.gripper.event_detected(whicharm) == 3
            delta_time = rospy.get_rostime().to_sec() - start
            timeout=delta_time > move_duration
            not_moving = not (self.ctrl.is_moving(whicharm) )and  delta_time > 3
            continue_move = not(event_detected or timeout or not_moving)
        self.ctrl.cart_freeze_arm(whicharm)
      
        if timeout or not_moving:
            # did arm even move?
            at_start = self.new_ctrl.check_cartesian_done(start_pose,.15,.0)
            # assume if arm moved more than .15 m then we got close enough to
            #goal to place otherwise return false since we got stuck hovering 
            return not at_start
            """
            if timeout:
                rospy.loginfo("Timed out")
            else:
                rospy.loginfo("arm stopped moving before timeout")
            rospy.loginfo("status of arm at start = %s " % at_start)
            return not at_start 
            """
        else:
            rospy.loginfo("gripper event detected!")
            return True

    #move to a Cartesian pose goal
    def move_cartesian_step(self, whicharm, pose, frame_id="base_link",\
            move_duration=5.0, ik_timeout=5.0, blocking=True):

        #result = self.new_ctrl.move_cartesian(pose, blocking=blocking)
        result = self.ctrl.cart_movearm(whicharm, pose, frame_id, move_duration, \
                ik_timeout, blocking) 
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
    parser = ArgumentParser(description="which script should we run for pick and place demo")
    parser.add_argument("-t", "--time", help="complete time trials", action="store_true", dest="time")
    parser.add_argument("-c", "--continuous", help="run continuously", action="store_true", dest="continuous")
    parser.add_argument("--place", help="only do place action", action="store_true")
    parser.add_argument("--pick", help="only do pick action", action = "store_true")


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
            if not args.continuous:
                break

