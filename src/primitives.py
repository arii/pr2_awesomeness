import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import numpy as np
from awesome_arm_controller import ArmController
from argparse import ArgumentParser

class Primitives:
    block_size = 1e-3 * 13.0 # mm
    W,H = 10.0, 20.0 # tetris grid units
    z_above = 0.1
    z_down = 0.02
    sqr2 = np.sqrt(2.0)/2.0
    sqr7 = np.sqrt(7.0)/4.0

    vert = [-.5, .5, .5, .5]
    horz = [0, sqr2, 0, sqr2]
    rot_r = [-sqr7, -.25, sqr7, -.25]
    rot_l = [sqr7, -.25, -sqr7, -.25]

    init_pose = {
        'r': [-0.73, -0.17, -1.60, -1.46, -32.69, -1.31, -16.94],
        'l': [ 0.71, -0.04, 1.56, -1.46, -4.72, -1.36, 3.86]}
    min_time = 2 
    max_time = 6.0
    frame = 'tetris'

    def __init__(self, whicharm='l'):
        self.arm = ArmController()
        self.whicharm = whicharm
        self.move_arm_to_side()
      
    def move_arm_to_side(self):
        self.arm.joint_movearm(self.whicharm, self.init_pose[self.whicharm],
                3, True)
   
    def lift_arm(self):
        curr_pos  = self.arm.get_cartesian_pose(self.frame)[self.whicharm]
        curr_pos[2] += self.z_above
        goal =[ self.stiff_pose(curr_pos, 0.1) + [0.5*self.min_time], 
                 self.stiff_pose(curr_pos, 0.5) + [self.min_time] ]
        self.arm.cart_movearm(self.whicharm, goal , self.frame, True)
   
    def stiff_pose(self, pose, rating = 0.2):
        pose_force = int(100*rating)
        torque_force = int(50*rating)
        return list(pose) + [pose_force]*3 + [torque_force]*3 + [False]*6
    
       
    def distance_between_two_pts(self, p1,p2):
        pts = zip(p1,p2)
        dx = [(x1 - x2)**2 for (x1,x2) in pts]
        return sum(dx)  

    def clip_time(self, t):
        if t < self.min_time:
            return self.min_time
        elif t > self.max_time:
            return self.max_time
        else:
            return t

    def approach_pose(self, pose):
        curr_pose = self.arm.get_cartesian_pose(self.frame)[self.whicharm]
        time = self.clip_time(10*self.distance_between_two_pts(curr_pose, pose))
        curr_pose[2] += self.z_above
        middle_pose = list(pose)
        middle_pose[2] += self.z_above

        traj = [
                self.stiff_pose(curr_pose, 0.1) + [time*.1],
                self.stiff_pose(curr_pose, 0.5) + [time*.2],
                self.stiff_pose(middle_pose, 0.7) + [time*.8],
                self.free_push(pose) + [time]
                ]

        self.arm.cart_movearm(self.whicharm, traj, self.frame, True)


        
    def do_force_control(self, cmd, timeout=2.0):
        self.approach_pose(cmd[:7])
        ros_timeout = rospy.Time.now() + rospy.Duration(timeout)
        self.arm.cart_movearm(self.whicharm, [cmd + [timeout]], self.frame, False)
        while rospy.Time.now() < ros_timeout:
            rospy.sleep(0.1)
            if not self.arm.is_moving(self.whicharm):
                self.arm.cancel_goal(self.whicharm)
                break

        self.lift_arm()

    def free_space_push_down(self, pos1, pos2):
        pose1 =  list(pos1) + self.horz
        pose2 =  list(pos2) + self.horz
        time = self.clip_time(self.distance_between_two_pts(pose1,pose2))
        self.approach_pose(pose1)

        traj = [
                self.free_push(pose1) + [time*.1],
                self.free_push(pose2) + [time]
                ]

        self.arm.cart_movearm(self.whicharm, traj, self.frame, True)


    def free_push(self, pose):  # position controlled
        fx,fy,fz = [300]*3  # max stiffness
        ox,oy,oz = [10]*3
        fz = -5  # force in downard direction
        states = [False]*6
        states[2] = True # use force control for Z
        return list(pose) + [fx,fy,fz,ox,oy,oz] + states 

    def right_contact_slide_down(self, pos):
        pose = list(pos) + self.rot_r
        #use force control
        fx = -7
        fy =  -1 
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)
    
    def left_contact_slide_down(self, pos):
        pose = list(pos) + self.rot_l
        #use force control
        fx = -7
        fy =  2
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)
    
    def right_contact_slide_right(self, pos):
        pose = list(pos) + self.rot_r
        #use force control
        fx = -2
        fy =  -5
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    def push_down(self, pos):
        pose =  list(pos) + self.horz
        #use force control
        fx = -7
        fy =  0 
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    
    def push_right(self, pos):
        pose =  list(pos) + self.vert
        #use force control
        fx = 0
        fy =  -5 
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    def push_left(self, pos):
        pose =  list(pos) + self.vert
        #use force control
        fx = 0
        fy =  5 
        fz = -2
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)



    def convert_xy(self, (x,y) ):
        return (x*self.block_size, y*self.block_size, self.z_down)

      

if __name__=="__main__":
    
    parser = ArgumentParser(description="hello ari")
    parser.add_argument("--init", action="store_true")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--gripper", action="store_true")
    args = parser.parse_args()

        
    rospy.init_node("tetris")

 
    tetris = Primitives()

    
    above_center = tetris.convert_xy( (tetris.W/2, tetris.H*1.2))
    left_corner  = tetris.convert_xy( (tetris.W*.1, tetris.H*.2))
    middle_center = tetris.convert_xy( (tetris.W/2, tetris.H/2) )


    #tetris.push_down(above_center)
    #tetris.push_right(left_corner)
    
    #tetris.free_space_push_down(above_center, middle_center)
    #tetris.push_right(middle_center)

    tetris.right_contact_slide_right(left_corner)



