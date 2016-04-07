import numpy as np
import rospy
import roslib; 
roslib.load_manifest("pr2_awesomeness")
import ee_cart_imped_action
import tf

import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction,\
        Pr2GripperCommandGoal, Pr2GripperCommand
     
class Arm:
    def __init__(self, tf_listener = None):
        self.control= {
                'l': ee_cart_imped_action.EECartImpedClient("left_arm"),
                'r': ee_cart_imped_action.EECartImpedClient("right_arm")
                }
        self.gripper = {\
                arm:  actionlib.SimpleActionClient(arm+"_gripper_controller/gripper_action",Pr2GripperCommandAction) for arm in ["l", "r"]
              }
        self.gripper['r'].wait_for_server()

        # for gripper listener:
        if tf_listener== None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

    def command_gripper(self, whicharm, position, max_effort=-1.0):

        self.gripper[whicharm].send_goal(Pr2GripperCommandGoal(
                    Pr2GripperCommand(position = position, max_effort = max_effort)))
        """client.wait_for_result()
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        print self.gripper
        self.gripper[whicharm].send_goal(goal)
        #if blocking:
        #    self.gripper[whicharm].wait_for_result()
        #    return 
        """

    #return the current Cartesian pose of the gripper
    def return_cartesian_pose(self, whicharm, frame = 'base_link'):
        start_time = rospy.get_rostime()
        current_time = rospy.get_rostime()
        link = "_gripper_tool_frame"
        while current_time - start_time < rospy.Duration(5.):
            current_time = rospy.get_rostime()
            try:
                t = self.tf_listener.getLatestCommonTime(\
                        frame, whicharm+link)
                (trans, rot) = self.tf_listener.lookupTransform(\
                        frame, whicharm+link, t)
                return list(trans) + list(rot)
            except (tf.Exception, tf.ExtrapolationException):
                rospy.sleep(0.5)
                current_time = rospy.get_rostime()
                rospy.logerr(\
                "waiting for a tf transform between %s and %s"%\
                (frame, whicharm+link))
        rospy.logerr("return_cartesian_pose waited 10 seconds tf\
                transform!  Returning None")
        return None



    def cancel_goal(self,whicharm):
        rospy.loginfo("canceling goals for %s" % whicharm)
        self.control[whicharm].cancelGoal()
        self.control[whicharm].resetGoal()

    def cart_movearm(self,whicharm, poses, frame_id, blocking=False):
        self.cancel_goal(whicharm)
        for pose in poses:
            pose = list(pose) + [ frame_id]
            self.control[whicharm].addTrajectoryPoint(*pose)
        self.control[whicharm].sendGoal(wait=blocking)
        
        if blocking:
            self.control[whicharm].cancelGoal()
            self.control[whicharm].resetGoal()
        return 

    def stiff_pose(self, pose):
        return list(pose) + [800]*3 + [30]*3 + [False]*6

if __name__ == "__main__":

    rospy.init_node("simpletest")

    arm = Arm()
    rospy.loginfo("getting pose")

    arm.command_gripper('r', 0)
    arm.command_gripper('l', 0)
    #initialize to get table pose
    init_table = arm.return_cartesian_pose('r', "base_link")

    move_left = list(init_table)
    move_left[1] += 0.5
    
    fx,fy,fz = [800]*3  # max stiffness
    ox,oy,oz = [30]*3
    fz = -2  # force in downard direction
    states = [False]*6
    states[2] = True # use force control for Z
   
    move_left_full = move_left + [fx,fy,fz,ox,oy,oz] + states 
    move_right_full = list(move_left_full)
    move_right_full[1] -= 0.5

    poses = [move_left_full + [5.0], move_right_full + [10.0]]
    arm.cart_movearm('r', poses , "base_link", True)
    raw_input("done?") 
    print "cartesian pose is: %s" % arm.return_cartesian_pose('r', 'base_link')
    #print "joint angles are: %s" %  ctrl.get_joint_angles()
