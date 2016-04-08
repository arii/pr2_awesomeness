#! /usr/bin/python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import tf
import tf.transformations as transf
import numpy as np
class Base:
  base_rot_vel = 0.30
  base_rot_thr = 0.03 
  base_rot_gain = 7.0
  base_trans_vel = 0.20
  base_trans_thr = 0.03 
  base_trans_gain = 7.0
  speedFactor =1.0
  verbose = True

  def __init__(self):
    self.rate = rospy.Rate(120) # 30hz
    self.tf_listener = tf.TransformListener()
    self.initTransform = self.getBaseTransform()
    self.initTransformInverse = transf.inverse_matrix(self.initTransform)
    self.basePub = rospy.Publisher('/base_controller/command', Twist)
    rospy.sleep(1)

 
  def getBaseTransform(self):
    now = rospy.Time()
    self.tf_listener.waitForTransform('/base_link','/odom_combined', now,
                                      rospy.Duration(10))
    pos, quat = self.tf_listener.lookupTransform('/odom_combined','/base_link', now)
    trans = transf.translation_matrix(pos)
    rot = transf.quaternion_matrix(quat)
    return np.dot(trans, rot)

  def getBasePose(self):
    now = self.getBaseTransform()
    rel = np.dot(self.initTransformInverse, now)
    ans = [rel[0,3], rel[1,3], transf.euler_from_matrix(rel)[2]]
    return ans

  def getBaseZ(self):
    z = self.getBaseTransform()[2,3]
    return z
  
  def rotateBase(self, ta, thr = None, timeout=10):
    ta = fixAnglePlusMinusPi(ta)
    cmd = Twist()
    (cx, cy, ca) = self.getBasePose()
    rospy.loginfo('R: Start base '+str((cx, cy, ca)))
    final_angle = fixAnglePlusMinusPi(ta + ca)
    delta = fixAnglePlusMinusPi(final_angle -ca)
    count = 0

    max_vel = self.base_rot_vel * self.speedFactor
    thr = thr or self.base_rot_thr
    final_time = rospy.get_time() + timeout

    while abs(delta) > thr:
        v = self.base_rot_gain*delta
        if abs(v) > max_vel:
            v = max_vel if v > 0 else -max_vel
        cmd.angular.z = v
        self.basePub.publish(cmd)
        self.rate.sleep()
        (cx, cy, ca) = self.getBasePose()
        delta = fixAnglePlusMinusPi(final_angle - ca)
        count += 1
        if self.verbose and count%30 == 0:
            print (cx, cy, ca), delta, v
        if rospy.get_time() > final_time:
            rospy.loginfo("stopping rotate due to timeout")
            break

    self.basePub.publish(Twist())   # stop
    if self.verbose:
        print 'Final base', self.getBasePose()
    rospy.loginfo("Success on base rot move")

  def rotateBaseToAngle(self, ta, thr = None):
    ta = fixAnglePlusMinusPi(ta)
    cmd = Twist()
    (cx, cy, ca) = self.getBasePose()
    rospy.loginfo('R: Start base '+str((cx, cy, ca)))
    delta = fixAnglePlusMinusPi(ta - ca)
    count = 0

    max_vel = self.base_rot_vel * self.speedFactor
    thr = thr or self.base_rot_thr
    while abs(delta) > thr:
        v = self.base_rot_gain*delta
        if abs(v) > max_vel:
            v = max_vel if v > 0 else -max_vel
        cmd.angular.z = v
        self.basePub.publish(cmd)
        self.rate.sleep()
        (cx, cy, ca) = self.getBasePose()
        delta = fixAnglePlusMinusPi(ta - ca)
        count += 1
        if self.verbose and count%30 == 0:
            print (cx, cy, ca), delta, v

    self.basePub.publish(Twist())   # stop
    if self.verbose:
        print 'Final base', self.getBasePose()
    rospy.loginfo("Success on base rot move")
      
  def translateBaseToPos(self, tx, ty, ta, thr = None):
    cmd = Twist()
    (cx, cy, ca) = self.getBasePose()
    dist = np.sqrt((tx - cx)**2 + (ty - cy)**2)
    count = 0
    rospy.loginfo('T: base distance '+str(dist))

    thr = thr or self.base_trans_thr
    while dist > thr:
        sa = np.sin(ca)
        ca = np.cos(ca)
        dx = (tx - cx)
        dy = (ty - cy)
        dx_rel =  ca*dx + sa*dy
        dy_rel = -sa*dx + ca*dy
        vx = self.base_trans_gain*dx_rel
        vy = self.base_trans_gain*dy_rel
        v = np.sqrt(vx**2 + vy**2)
        max_vel = self.base_trans_vel * self.speedFactor
        if v > max_vel:
            slow = max_vel/v
            vx *= slow
            vy *= slow
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.basePub.publish(cmd)
        self.rate.sleep()
        (cx, cy, ca) = self.getBasePose()
        dist = np.sqrt((tx - cx)**2 + (ty - cy)**2)
        count += 1
        if self.verbose and count%30 == 0:
            print (cx, cy, ca), dist, v

    self.basePub.publish(Twist())   # stop
    if self.verbose:
        print 'Final base', self.getBasePose()
    rospy.loginfo("Success on base trans move")

  def moveBaseToPose(self, pose, path = []):
    rospy.loginfo('moving base to '+str(pose))
    if path:
        rot_thr = self.base_rot_thr * 2
        trans_thr = self.base_trans_thr * 2
        for (x, y, a) in path:
            self.translateBaseToPos(x, y, a, trans_thr)
            self.rotateBaseToAngle(a, rot_thr)
        pose = path[-1]
    (x, y, a) = pose
    self.translateBaseToPos(x, y, a)
    self.rotateBaseToAngle(a)
    return 'goal'
def fixAnglePlusMinusPi(a):
    """ hard = 0.035

    A is an angle in radians;  return an equivalent angle between plus
    and minus pi
    """
    if a > np.pi:
        return fixAnglePlusMinusPi(a - 2.0* np.pi)
    elif a < -np.pi:
        return fixAnglePlusMinusPi(a + 2.0*np.pi)
    else:
        return a




if __name__=="__main__":
  rospy.init_node('move_the_base', anonymous=True)
  base = Base()
  print base.getBasePose()
  base.rotateBase(1.57)
  #base.moveBaseToPose((.1,.1,0))#np.pi/2))
