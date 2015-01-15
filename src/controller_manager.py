from __future__ import division
import roslib; roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, Vector3Stamped
from pr2_mechanism_msgs.srv import SwitchController, ListControllers
import actionlib
import math
import pdb
import copy
from interpolated_ik_motion_planner import ik_utilities
from object_manipulator.convert_functions import *
from actionlib_msgs.msg import GoalStatus
from pr2_gripper_reactive_approach.controller_params import *
import threading
import scipy.linalg

##class to start/stop/switch between joint and Cartesian controllers
class ControllerManager():

  #whicharm is 'r' or 'l'
  def __init__(self, whicharm, tf_listener = None, ): 
    self.whicharm = whicharm


    rospy.loginfo("controller manager waiting for pr2_controller_manager services to be there")
    switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
    list_controllers_serv_name = 'pr2_controller_manager/list_controllers'
    self.wait_for_service(switch_controller_serv_name)
    self.wait_for_service(list_controllers_serv_name)
    self.switch_controller_service = \
        rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
    self.list_controllers_service = \
        rospy.ServiceProxy(list_controllers_serv_name, ListControllers)
    
    #move arm client is filled in the first time it's called
    self.move_arm_client = None
    
    #initialize a tf listener
    if tf_listener == None:
      self.tf_listener = tf.TransformListener()
    else:
      self.tf_listener = tf_listener


    #which Cartesian controller to use
    self.use_trajectory_cartesian = rospy.get_param('~use_trajectory_cartesian', 0)
    self.use_task_cartesian = rospy.get_param('~use_task_cartesian', 0)
    if self.use_trajectory_cartesian and self.use_task_cartesian:
      rospy.loginfo("can't use both trajectory and task controllers!  Defaulting to task")
      self.use_trajectory_cartesian = 0
    rospy.loginfo("starting cartesian controller")
    rospy.loginfo("controller_manager: use_trajectory_cartesian: "+str(self.use_trajectory_cartesian))
    rospy.loginfo("controller_manager: use_task_cartesian: "+str(self.use_task_cartesian))

    #names of the controllers
    self.cartesian_controller = self.whicharm + '_cart'
    self.joint_controller = self.whicharm+'_arm_controller'
    #XXXself.cartesian_controller = self.whicharm + "_arm_cart_imped_controller"

    #parameters for the Cartesian controllers    
    self.cart_params = JTCartesianTaskParams(self.whicharm, self.use_task_cartesian)

    #parameters for the joint controller
    self.joint_params = JointParams(self.whicharm)

    #services for the J-transpose Cartesian controller 
    cartesian_cmd_name = whicharm+'_cart/command_pose'
    self.cartesian_cmd_pub = rospy.Publisher(cartesian_cmd_name, PoseStamped) 

    #create an IKUtilities class object
    rospy.loginfo("creating IKUtilities class objects")
    if whicharm == 'r':
      self.ik_utilities = ik_utilities.IKUtilities('right', self.tf_listener)
    else:
      self.ik_utilities = ik_utilities.IKUtilities('left', self.tf_listener)
    rospy.loginfo("done creating IKUtilities class objects")
    #thread and params to send clipped/overshoot versions of the last Cartesian command
    self.dist_tol = .001
    self.angle_tol = .05
    self.overshoot_dist = 0.005
    self.overshoot_angle = 0.087
    self.timestep = 1./120.
    self.cartesian_command_lock = threading.Lock()
    self.cartesian_command_thread_running = False
    self.cartesian_desired_pose = self.get_current_wrist_pose_stamped('/base_link')
    self.cartesian_command_thread = threading.Thread(target=self.cartesian_command_thread_func)
    self.cartesian_command_thread.setDaemon(True)
    self.cartesian_command_thread.start()
    
    #load the Cartesian controllers if not already loaded
    rospy.loginfo("loading any unloaded Cartesian controllers")
    #self.start_cartesian_controllers()
    rospy.loginfo("done loading controllers")

    rospy.loginfo("done with controller_manager init for the %s arm"%whicharm)

  ##set the current desired Cartesian pose to the current gripper pose
  def set_desired_cartesian_to_current(self):
    current_pose = self.get_current_wrist_pose_stamped('/base_link')
    with self.cartesian_command_lock:
      self.cartesian_desired_pose = current_pose

  def get_controllers_status(self):
    resp = self.list_controllers_service()
    controllers = dict(zip(resp.controllers, resp.state))
    
    joint_loaded =  self.joint_controller in controllers
    cart_loaded =  self.cartesian_controller in controllers
    if not cart_loaded:
        rospy.loginfo("Cartesian controller not loaded! error")
    joint_running = controllers[self.joint_controller] =="running" \
        if joint_loaded else False
    cart_running = controllers[self.cartesian_controller] =="running" \
        if cart_loaded else False
    rospy.loginfo("joint controller running: %s, cart controller running: %s "\
        %(joint_running, cart_running))
    return joint_running, cart_running

  def start_cartesian_controllers(self):
    joint_running, cart_running = self.get_controllers_status() 
    self.set_desired_cartesian_to_current()
    rospy.loginfo("starting cartesian controller")
    if joint_running and cart_running:
        rospy.loginfo("cartesian and joint controllers both running."
          "stopping joint controller")
        self.switch_controller_service([], [self.joint_controller,],2)
    elif joint_running and not cart_running:
        rospy.loginfo("switching joint and cartesian controllers")
        self.switch_controller_service([self.cartesian_controller,],\
        [self.joint_controller,], 2)
    elif not joint_running and not cart_running:
        rospy.loginfo("starting cartesian controller")
        self.switch_controller_service([self.cartesian_controller,],[], 2)
    else:
        rospy.loginfo("cartesian controller already running")
   
  def start_joint_controllers(self):
    joint_running, cart_running = self.get_controllers_status() 
    rospy.loginfo("starting joint controller")
    if joint_running and cart_running:
        rospy.loginfo("cartesian and joint controllers both running."
          "stopping cart controller")
        self.switch_controller_service([], [self.cartesian_controller,],3)
    elif not joint_running and  cart_running:
        rospy.loginfo("switching joint and cartesian controllers")
        self.switch_controller_service([self.joint_controller,],\
        [self.cartesian_controller,], 3)
    elif not joint_running and not cart_running:
        rospy.loginfo("starting joint controller")
        self.switch_controller_service([self.joint_controller,],[], 3)
    else:
        rospy.loginfo("joint controller already running")

  ##wait for an action server to be ready
  def wait_for_action_server(self, client, name):
    while not rospy.is_shutdown():  
      rospy.loginfo("controller manager: waiting for %s to be there"%name)
      if client.wait_for_server(rospy.Duration(5.0)):
        break
    rospy.loginfo("controller manager: %s found"%name)  


  ##wait for a service to be ready
  def wait_for_service(self, name):
    while not rospy.is_shutdown():  
      rospy.loginfo("controller manager: waiting for %s to be there"%name)
      try:
        rospy.wait_for_service(name, 5.0)
      except rospy.ROSException:
        continue
      break
    rospy.loginfo("controller manager: %s found"%name)  

 

  ##return the current pose of the wrist as a PoseStamped
  def get_current_wrist_pose_stamped(self, frame = 'base_link'):
      (current_trans, current_rot) = self.return_cartesian_pose(frame)
      return create_pose_stamped(current_trans+current_rot, frame)

  ##clip pose to be no more than max_pos_dist or max_rot_dist away from the current gripper pose
  def clip_pose(self, pose, max_pos_dist = 0.01, max_rot_dist = math.pi/20., current_pose = None):
    
    if type(pose) == list:
        pose = create_pose_stamped(list(pose), 'base_link')


    if current_pose == None:
      current_pose = self.get_current_wrist_pose_stamped(pose.header.frame_id)
    current_mat = pose_to_mat(current_pose.pose)
    target_mat = pose_to_mat(pose.pose)

    target_to_current = current_mat**-1 * target_mat
    desired_trans = target_to_current[0:3, 3].copy()
    desired_trans_mag = scipy.linalg.norm(desired_trans)
    target_to_current[0:3, 3] = 0

    desired_angle, desired_axis, point = tf.transformations.rotation_from_matrix(target_to_current)
            
    frac_trans = math.fabs(desired_trans_mag / max_pos_dist)
    frac_rot = math.fabs(desired_angle / max_rot_dist)

    if frac_trans <= 1 and frac_rot <= 1:
      return pose
    frac = max(frac_rot, frac_trans)

    clamped_angle = desired_angle / frac
    clamped_trans = desired_trans / frac

    clamped_transformation = scipy.matrix(tf.transformations.rotation_matrix(clamped_angle, desired_axis))
    clamped_transformation[0:3, 3] = clamped_trans
    clamped_mat = current_mat * clamped_transformation
    clamped_pose = stamp_pose(mat_to_pose(clamped_mat), pose.header.frame_id)
    return clamped_pose


  ##thread function for sending clipped Cartesian commands
  def cartesian_command_thread_func(self):
    r = rospy.Rate(1./self.timestep)
    while not rospy.is_shutdown():
      with self.cartesian_command_lock:
        running = self.cartesian_command_thread_running
      if running:
        self.cartesian_desired_pose.header.stamp = rospy.Time.now()
        with self.cartesian_command_lock:
          desired_pose = self.cartesian_desired_pose
          overshoot_dist = self.overshoot_dist
          overshoot_angle = self.overshoot_angle
          dist_tol = self.dist_tol
          angle_tol = self.angle_tol

        current_pose = self.get_current_wrist_pose_stamped(desired_pose.header.frame_id)
        goal_pose = self.clip_pose(desired_pose, current_pose = current_pose)

        self.cartesian_cmd_pub.publish(desired_pose)
      #r.sleep()
 
  ##stop sending commands from the Cartesian command thread
  def stop_cartesian_commands(self):
    with self.cartesian_command_lock:
      self.cartesian_command_thread_running = False



  def command_cartesian(self, pose, frame_id = '/base_link'):
    with self.cartesian_command_lock:
      self.cartesian_command_thread_running = True
      self.cartesian_desired_pose = pose
      

  ##return the current Cartesian pose of the gripper
  def return_cartesian_pose(self, frame = 'base_link'):
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown():
      try:
        t = self.tf_listener.getLatestCommonTime(frame, self.whicharm+'_wrist_roll_link')
        (trans, rot) = self.tf_listener.lookupTransform(frame, self.whicharm+'_wrist_roll_link', t)
        break
      except (tf.Exception, tf.ExtrapolationException):
        rospy.sleep(0.5)
        current_time = rospy.get_rostime()
        rospy.logerr("waiting for a tf transform between %s and %s"%(frame, \
                                                   self.whicharm+'_wrist_roll_link'))
        if current_time - start_time > rospy.Duration(30.):
          rospy.logerr("return_cartesian_pose waited 30 seconds for a tf transform!  Returning None")
          return (None, None)
    return (list(trans), list(rot))




  ##check if we're near to a desired Cartesian pose (either PoseStamped or 7-list of position and quaternion
  #in 'base_link' frame) by pos_thres(m) and rot_thres (rad)
  def check_cartesian_near_pose(self, goal_pose, pos_thres, rot_thres):
    (goal_pos, goal_rot) = pose_stamped_to_lists(self.tf_listener, goal_pose, 'base_link')
    (pos_dist, rot_dist) = self.dist_from_current_pos_and_rot(goal_pos, goal_rot)
    if pos_dist < pos_thres and rot_dist < rot_thres:
      return 1
    return 0    


  ##return the distance and angle from the current Cartesian pose to a goal pose
  def dist_from_current_pos_and_rot(self, goal_pos, goal_rot, current_pos = None, current_rot = None):

    if current_pos == None or current_rot == None:
      (current_pos, current_rot) = self.return_cartesian_pose()
    
    #compute how far the wrist is translated from the goal
    diff = [x-y for (x,y) in zip(goal_pos, current_pos)]
    pos_diff = self.ik_utilities.vect_norm(diff)

    #compute how far the wrist is rotated from the goal
    norm_goal_rot = self.ik_utilities.normalize_vect(goal_rot)
    norm_current_rot = self.ik_utilities.normalize_vect(current_rot)
    rot_diff = self.ik_utilities.quat_angle(norm_current_rot, norm_goal_rot)
    #rospy.loginfo("pos_diff: %5.3f, rot_diff: %5.3f"%(pos_diff, rot_diff))

    return (pos_diff, rot_diff)


  ##check if both the Cartesian controllers think we're done and if we're close to where we want to be
  def check_cartesian_done(self, goal_pose, pos_thres, rot_thres, \
      current_pos = None, current_rot = None):
    (current_pos, current_rot) = self.return_cartesian_pose()
    done = self.check_cartesian_near_pose(goal_pose, pos_thres, rot_thres)
    if done:
        self.stop_cartesian_commands()
    return done


  
  def wait_cartesian_done(self, goal_pose, pos_thres = .2,\
    rot_thres=.1, timeout=rospy.Duration(10)):
      start_time = rospy.get_rostime()
      r = rospy.Rate(20)
      while not rospy.is_shutdown():
          if rospy.get_rostime() - start_time > timeout:
              rospy.loginfo("timed out")
              return False

          if self.check_cartesian_done(goal_pose, pos_thres, rot_thres):
              rospy.loginfo("got to the goal")
              return True
          
          r.sleep()


  def move_cartesian(self, goal_pose, collision_aware = 0, blocking = 1,
                     step_size = .005, pos_thres = .02, rot_thres = .1,
                     timeout = rospy.Duration(15.)):
    
    self.start_cartesian_controllers()
    if type(goal_pose) == list:
        goal_pose = create_pose_stamped(goal_pose, 'base_link')

    #go to the goal pose using the Cartesian controllers
    self.command_cartesian(goal_pose)
    if not blocking:
      return None
    #blocking: wait for the Cartesian controllers to get there or time out
    result = self.wait_cartesian_done(goal_pose, pos_thres, \
                 rot_thres, timeout = timeout)
    return result

#sample test cases
if __name__ == '__main__':
  rospy.init_node("asdfkl")
  cm = ControllerManager('l')
  (pos,rot) = cm.return_cartesian_pose()
  pos[0]+=.1
  desired_pose =create_pose_stamped([pos+rot])
  print desired_pose
  cm.command_cartesian(desired_pose)

