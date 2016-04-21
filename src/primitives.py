import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import numpy as np
from awesome_arm_controller import ArmController
from argparse import ArgumentParser
from geometry_msgs.msg import PoseArray
from object_manipulator.convert_functions import get_transform

class Primitives:
    block_size = 1e-3 * 13.0 # mm
    W,H = 10.0, 20.0 # tetris grid units
    z_above = 0.07
    z_down = 0.03
    sqr2 = np.sqrt(2.0)/2.0
    sqr7 = np.sqrt(7.0)/4.0

    vert = [-.5, .5, .5, .5]
    horz = [0, sqr2, 0, sqr2]
    #rot_r = [-sqr7, -.25, sqr7, -.25]
    rot_r = [-.25, sqr7, 0.25, sqr7]
    rot_l = [sqr7, -.25, -sqr7, -.25]

    init_pose = {
        #'r': [-0.73, -0.17, -1.60, -1.46, -32.69, -1.31, -16.94],
        #'l': [ 0.71, -0.04, 1.56, -1.46, -4.72, -1.36, 3.86]}
        'l': [ 0.95, 0.0, np.pi/2., -np.pi/2., -np.pi*1.5, -np.pi/2.0, np.pi],
        'r': [ -0.7, 0.0, -np.pi/2., -np.pi/2., -20.424894658897493, -np.pi/2.0, np.pi]}
    min_time = 1.0
    max_time = 5.0
    frame = 'tetris'

    def __init__(self, whicharm='l'):
        self.arm = ArmController()
        self.objects = None
        self.obj_sub = rospy.Subscriber("/table_publisher/object_poses", PoseArray, self.objCb)
        self.arm.command_torso(0.26)
        self.whicharm = whicharm
        self.move_arm_to_side()
      
    def move_arm_to_side(self):
        self.arm.joint_movearm(self.whicharm, self.init_pose[self.whicharm],
                3, True)
   
    def lift_arm(self, rating=1.0):
        rospy.sleep(0.1)
        curr_pos  = self.arm.get_cartesian_pose(self.frame)[self.whicharm]
        curr_pos[2] = self.z_above *rating
        goal =[ self.stiff_pose(curr_pos, 0.1) + [0.25*self.min_time*rating], 
                 self.stiff_pose(curr_pos, 0.5) + [0.5*self.min_time*rating],
                 self.stiff_pose(curr_pos, 1.0) + [self.min_time*rating] ]
        self.arm.cart_movearm(self.whicharm, goal , self.frame, True)
        #raw_input("done lift")
   
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
        time = self.clip_time(15*self.distance_between_two_pts(curr_pose, pose))
        curr_pose[2] = self.z_above
        middle_pose = list(pose)
        middle_pose[2] = self.z_above

        traj = [
                self.stiff_pose(curr_pose, 0.1) + [time*.1],
                self.stiff_pose(curr_pose, 1.0) + [time*.2],
                self.stiff_pose(middle_pose, 1.5) + [time*.8],
                self.free_push(pose, 1.5) + [time]
                ]

        self.arm.cart_movearm(self.whicharm, traj, self.frame, True)
        rospy.sleep(0.1)
        #final_pose = self.arm.get_cartesian_pose(self.frame)[self.whicharm]
        #dist = self.distance_between_two_pts(pose, final_pose)

        
    def do_force_control(self, cmd, timeout=4.0):
        self.approach_pose(cmd[:7])
        rospy.sleep(0.1)

        curr_pose = self.arm.get_cartesian_pose(self.frame)[self.whicharm]

        ros_timeout = rospy.Time.now() + rospy.Duration(timeout)
        self.arm.cart_movearm(self.whicharm, [cmd + [timeout]], self.frame, False)
        rospy.sleep(0.2)
        while rospy.Time.now() < ros_timeout and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if not self.arm.is_moving(self.whicharm):
                self.arm.cancel_goal(self.whicharm)
                break
        
        """ 
        final_pose = self.arm.get_cartesian_pose(self.frame)[self.whicharm]
        if self.distance_between_two_pts(curr_pose, final_pose) < self.block_size:
            ros_timeout = rospy.Time.now() + rospy.Duration(timeout)
            self.arm.cart_movearm(self.whicharm, [cmd + [timeout]], self.frame, False)
            rospy.sleep(1.0)
            while rospy.Time.now() < ros_timeout:
                rospy.sleep(0.1)
                if not self.arm.is_moving(self.whicharm):
                    self.arm.cancel_goal(self.whicharm)
                    break
        """

        self.lift_arm()

    def free_space_push_down(self, pos1, pos2):
        pose1 =  list(pos1) + self.horz
        pose2 =  list(pos2) + self.horz
        time = self.clip_time(15*self.distance_between_two_pts(pose1,pose2))
        self.approach_pose(pose1)

        traj = [
                self.free_push(pose1) + [time*.1],
                self.free_push(pose2) + [time]
                ]

        self.arm.cart_movearm(self.whicharm, traj, self.frame, True)
        self.lift_arm(1.)


    def free_push(self, pose, rating=1.0):  # position controlled
        fx,fy,fz = [300*rating]*3  # max stiffness
        ox,oy,oz = [10*rating]*3
        fz = -5*rating  # force in downard direction
        states = [False]*6
        states[2] = True # use force control for Z
        return list(pose) + [fx,fy,fz,ox,oy,oz] + states 

    def right_contact_slide_down(self, pos, rating=1.0):
        pose = list(pos) + self.rot_r
        #use force control
        fx = -7*rating
        fy =  -3 * rating
        fz = -2* rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)
    
    def left_contact_slide_down(self, pos, rating=1.0):
        pose = list(pos) + self.rot_l
        #use force control
        fx = -7*rating
        fy =  2*rating
        fz = -2*rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)
    
    def right_contact_slide_right(self, pos, rating=1.0):
        pose = list(pos) + self.rot_r
        #use force control
        fx = -2*rating
        fy =  -5*rating
        fz = -2*rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    def push_down(self, pos, rating=1.0):
        pose =  list(pos) + self.horz
        #use force control
        fx = -7*rating
        fy = 300*rating
        fz = -3*rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[True, False, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    
    def push_right(self, pos, rating=1.0):
        pose =  list(pos) + self.vert
        #use force control
        fx = 400*rating
        fy =  -5 *rating
        fz = -3*rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[False, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)

    def push_left(self, pos, rating=1.0):
        pose =  list(pos) + self.vert
        #use force control
        fx = 300*rating
        fy =  5 *rating
        fz = -3*rating
        #keep gripper rotated
        ox,oy,oz = [30]*3
        states =[False, True, True] +  [False]*3
        cmd = list(pose) + [fx,fy,fz,ox,oy,oz] + states 
        self.do_force_control(cmd)



    def convert_xy(self, (x,y) ):
        return (x*self.block_size, y*self.block_size, self.z_down)

    def objCb(self, msg):
        try:
            T =  get_transform(self.arm.tf_listener, "base_link", "tetris")#.getI()
        except:
            return
        tetris_pts = []
        
        for pt in msg.poses:
            p = pt.position.x, pt.position.y, pt.position.z
            tetris_pt = self.transform_pts(T, p)
            tetris_pt[2] = self.z_down # remap z to 0
            #tetris_pts.append(tetris_pt)

            grid_pts =[ x/self.block_size  for x in tetris_pt[:2] ]
            if self.in_grid(grid_pts):
                tetris_pts.append(self.clip_grid(grid_pts))
            #print grid_pts
        tetris_pts = sorted(tetris_pts, reverse=True, key=lambda x: x[1]) # sort by largest y first
        if len(tetris_pts) > 0:
            self.objects = tetris_pts
    def in_grid(self, (x,y)):
        if x < -2 : return False
        if y < -2: return False

        if x > self.W*1.2: return False
        if y > self.H*1.5: return False

        return True

    def clip_grid(self, (x,y)):
        if x > self.W:
            x = self.W*0.85
        elif x < 0:
            x = 0
        if y > self.H*1.5:
            y = self.H*1.5
        elif y < 0:
            y = 0
        return [x,y]


    def transform_pts(self, T, p):
        v = np.matrix(list(p) + [1.0])
        pt =  T*v.T 
        return [float(p) for p in list(pt)[:3]]




    
    
    
      

if __name__=="__main__":
    
    parser = ArgumentParser(description="hello ari")
    parser.add_argument("--init", action="store_true")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--gripper", action="store_true")
    args = parser.parse_args()

        
    rospy.init_node("tetris")

    

 
    tetris = Primitives()
    

    above_center = tetris.convert_xy( (tetris.W/2, tetris.H*1.2))
    left_corner  = tetris.convert_xy( (0,0))
    middle_center = tetris.convert_xy( (tetris.W/2, 0.7*tetris.H) )
    
    #given direction of push and object location.. figure out when paddle should be?
    # instead it will just be object left, object right etc.
    
    left_corner  = tetris.convert_xy( (0,0) )

    def find_object():
        tetris.objects = None
        rospy.sleep(.2)
        while tetris.objects == None and not rospy.is_shutdown():
            rospy.loginfo("waiting to detect object")
            rospy.sleep(0.2)
    
    def get_above_pose():
        find_object()
        objat = list(tetris.objects[0])
        objat[1] += 4.5
        above_center = tetris.convert_xy(objat)
        return above_center
    
    def get_left_pose():
        objat = list(tetris.objects[0])
        objat[0] = 2
        left = tetris.convert_xy(objat)
        return left


   
    def push_block_down(middle=False):

        above_pose = get_above_pose()

        if middle:
            middle_center = list(above_pose)
            middle_center[1] = 0.7*tetris.H*tetris.block_size
            tetris.free_space_push_down(above_pose,middle_center)
        else:
            tetris.push_down(above_pose, 1.2)

    def push_block_right():
        left = get_left_pose()
        tetris.push_right(left)

    def slide_block_on_right():
        above_pose = get_above_pose()
        tetris.right_contact_slide_down(above_pose)

    
    
    def push_block_to_corner():
        push_block_down(middle=False)
        push_block_right()

    def block_ontop_corner():
        push_block_down(middle=True)
        push_block_right()
        slide_block_on_right()


    for i in range(10):
        push_block_to_corner()
        tetris.move_arm_to_side()

        block_ontop_corner()
        tetris.move_arm_to_side()

        
        raw_input("repeat test")


    """ 
        objat = list(tetris.objects[0])
        objat[1] += 4.5
        above_center = tetris.convert_xy(objat)



        # push block to corner
        tetris.push_down(above_center, 1.2)

        if tetris.objects[0][1] > 10: 
            tetris.move_arm_to_side()
            print "object not low enough!"
            continue

        tetris.push_right(left_corner)
        

        tetris.move_arm_to_side()
   

    tetris.push_down(above_center, 1.2)
    tetris.push_right(left_corner)

    tetris.move_arm_to_side()
    raw_input("start")
    # push block on top of corner block
    tetris.free_space_push_down(above_center, middle_center)
    tetris.push_right(middle_center)
    tetris.right_contact_slide_down(middle_center)
    

    """

