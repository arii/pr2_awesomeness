import roslib; roslib.load_manifest('pr2_awesomeness')
import rospy 
import actionlib
from object_manipulator.convert_functions import *


  ##check if we're near to a desired Cartesian pose (either PoseStamped or 7-list of position and quaternion
  #in 'base_link' frame) by pos_thres(m) and rot_thres (rad)
def check_cartesian_near_pose(current_pose, goal_pose, pos_thres, rot_thres, tf_listener):
    goal_pos = goal_pose[:3]
    goal_rot = goal_pose[3:7]
    curr_pos = current_pose[:3]
    curr_rot = current_pose[3:]
    diff = [(x-y)**2 for (x,y) in zip(goal_pos, curr_pos)]
    return sum(diff) < pos_thres
 

##wait for an action server to be ready
def wait_for_action_server(client, name, timeout=5.0):
    while not rospy.is_shutdown():  
        rospy.loginfo("controller manager: waiting for %s to be there"%name)
        if client.wait_for_server(rospy.Duration(timeout)):
            break
        rospy.loginfo("controller manager: %s found"%name)  

##wait for a service to be ready
def wait_for_service(name, timeout=5.0):
    while not rospy.is_shutdown():  
        rospy.loginfo("controller manager: waiting for %s to be there"%name)
        try:
            rospy.wait_for_service(name, timeout)
        except rospy.ROSException:
            continue
        break
    rospy.loginfo("controller manager: %s found"%name)  


