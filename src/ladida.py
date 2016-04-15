import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import ee_cart_imped_action
from pr2_controller_manager import Pr2ControllerManager
from pick_and_place_manager import PickAndPlaceManager
from object_manipulator.convert_functions import lists_to_pose_stamped, get_transform, mat_to_pos_and_quat
import numpy as np
from awesome_arm_controller import ArmController
from tf_transformer import TfTransformer
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf
from broadcast_tetris_tf import TableBroadcaster
from argparse import ArgumentParser

"""

tetris = [(180, 80), (180,332), (28,332), (28,80)]
sqr2 = np.sqrt(2.0)/2.0
sqr7 = np.sqrt(7.0)/4.0
vert =  [0,sqr2, 0, sqr2]
horz =  [0.5, 0.5, -0.5, 0.5]
rot_r=  (-sqr7, 0.25, sqr7, 0.25)
rot_l=  (-.25, sqr7,0.25, sqr7)
blank_T = np.matrix(tf.transformations.quaternion_matrix( (0,0,0,1)))
"""
sqr2 = np.sqrt(2.0)/2.0
sqr7 = np.sqrt(7.0)/4.0

vert = [-.5, .5, .5, .5]
horz = [0, sqr2, 0, sqr2]
rot_r = [-sqr7, -.25, sqr7, -.25]
rot_l = [sqr7, -.25, -sqr7, -.25]


init_pose = {
    'r': [-0.73, -0.17, -1.60, -1.46, -32.69, -1.31, -16.94],
    'l': [ 0.71, -0.04, 1.56, -1.46, -4.72, -1.36, 3.86]}
z_above = .100 # hover over tetris board
z_down = 0
interpolate = 25
min_time = .1
W,H =  10.0,20.0
block_size =1e-3* 13.0 # mm



def move_arms_to_side(arm):
    arm.joint_movearms(init_pose)

def move_arm_to_side(arm, whicharm, move_duration=1, blocking=True):
    arm.joint_movearm(whicharm, init_pose[whicharm], move_duration, blocking)

def stiff_pose(pose, rating = 0.2):
    pose_force = int(100*rating)
    torque_force = int(50*rating)
    return list(pose) + [pose_force]*3 + [torque_force]*3 + [False]*6

def right_slide_down(pose):
    pos = pose[:3]
    pose = list(pos) + rot_r
    
    #use force control
    fx = -7
    fy =  -1
    fz = -2
    #keep gripper rotated
    ox,oy,oz = [30]*3
    states =[True, True, True] +  [False]*3

    return list(pose) + [fx,fy,fz,ox,oy,oz] + states 


def push_down(pose):
    
    fx,fy,fz = [300]*3  # max stiffness
    ox,oy,oz = [10]*3
    fz = -5  # force in downard direction
    states = [False]*6
    states[2] = True # use force control for Z

    return list(pose) + [fx,fy,fz,ox,oy,oz] + states 

def get_table(arm, next_frame ="tetris"):
    tf_transformer = TfTransformer(arm.tf_listener)
    T_base_table =  get_transform(arm.tf_listener, "base_link", next_frame).getI()
    return T_base_table

def point_transformed( T, pos):
    offset = blank_T
    offset[0:3,3] = np.matrix(pos).T
    new_pose = T * offset
    trans, rot = mat_to_pos_and_quat(new_pose)
    return trans,rot

def get_tetris_traj(T, px,py, direction, final=None):

    W,H =  10.0,20.0
    block_size = 13.0 # mm

    z_above = 100 # hover over tetris board
    z_down = 20
    interpolate = 4
    min_time = 2
    final_time =1
    start_time = 2

    
    if direction == "down":
        quat = horz
        final_x = px
        final_y = 0

        release =(0, block_size)

    elif direction == "up": 
        quat = horz
        final_x = px
        final_y = H
        release =(0, -block_size)

    elif direction == "right":
        quat = vert
        final_x = W
        final_y = py
        release = (-block_size, 0)

    elif direction == "left":
        quat = vert
        final_x = 0
        final_y = py
        release = (block_size, 0)
    if final != None:
        final_x, final_y = final

    x_traj = np.linspace(px*block_size, final_x*block_size, interpolate)
    y_traj = np.linspace(py*block_size, final_y*block_size, interpolate)
    
    # determine how much time it should take
    dist =.1*( (final_x - px )**2 + (final_y - py)**2 )**.5
    dist = max(min_time, dist) 
    t_traj = np.linspace(start_time, dist + start_time, interpolate+ 1)
    

    middle = zip(x_traj, y_traj,  [z_down]*interpolate)
    middle = [list(a) for a in middle]
    #transform into base link

    middle = [point_transformed(T, [1e-3*p for p in m])[0] for m in middle]
    middle.append(list(middle[-2])) #back up a bit
    
    start_up  =list( middle[0])
    start_up[2] += z_above*1e-3 # start above
    start_down = list(middle[0]) # stiff pose to table


    final = list(middle[interpolate/3])
    #final[2] += z_above*1e-3 # move up

    quat = list(quat)
    
    
    # add start times, quaternions, forces to each pose
    start = [stiff_pose(start_up + quat) + [start_time*.75],
             stiff_pose(start_down + quat) + [start_time]]
    middle = [push_down(middle[i] + quat ) + [t_traj[i]]\
            for i in range(interpolate + 1)]
    final =stiff_pose(final + quat) + [final_time + dist + start_time]
    
    trajectory = start + middle + [final]
    return trajectory, release
  

def distance_between_two_pts( p1,p2):
    pts = zip(p1,p2)
    dx = [(x1 - x2)**2 for (x1,x2) in pts]
    return sum(dx)


def approach_traj(arm, T, (x,y), direction):
    if direction=="up" or direction=="down":
        quat = horz
    else:
        quat = vert
    
    curr_pos, curr_quat = arm.get_cartesian_pose('tetris')['l']
    des_pos = x,y,z_above
    
    time = distance_between_two_pts(curr_pos)


    traj = [stiff_pose(start_up + quat) + [start_time*.75],
             stiff_pose(start_down + quat) + [start_time]]




def release_traj(arm):
    curr_pos  = arm.get_cartesian_pose('tetris')['l']
    curr_pos[2] += 0.1
    goal = stiff_pose( curr_pos) + [1.0]
    return [goal]



def do_tetris_prim(arm, T,prim):
    (x,y), direction, final  = prim
    goal, release = get_tetris_traj(T, x,y, direction, final)
    arm.cart_movearm('l', goal, 'base_link', True)
    goal2 = release_traj(arm,T,release)
    arm.cart_movearm('l', release_traj(arm, T, release), 'tetris', True)

def test_hover2(arm):
    """
    W,H =  130.0,250.0
    vert  = arm.get_cartesian_pose('tetris')['l'][3:]
    corners = [(0,0), (W,0), (W,H), (0, H), (0,0)]
    for (x,y) in corners:
        pos = [1e-3*p for p in [x,y,z_above] ]
        goal = stiff_pose( pos+ list(vert), .6 ) + [3.0]
        arm.cart_movearm('l', [goal], "tetris", True)
    
    quats = [vert, horz, rot_r, rot_l]
    cur_pose = arm.get_cartesian_pose('base_link')['l']
    pos = cur_pose[:3]
    for quat in quats:
        goal = stiff_pose( pos+ list(quat), .6 ) + [3.0]
        arm.cart_movearm('l', [goal], "base_link", True)

        print arm.get_cartesian_pose('tetris')['l'][3:]
        raw_input("next pose")

    """
    horz = [-.5, .5, .5, .5]
    vert = [0, sqr2, 0, sqr2]
    rot_r = [-sqr7, -.25, sqr7, -.25]
    rot_l = [sqr7, -.25, -sqr7, -.25]
   

    cur_pose = arm.get_cartesian_pose('tetris')['l']
    pos = cur_pose[:3]
    quats = [vert, horz, rot_r, rot_l]

    for quat in quats:
        goal = stiff_pose( pos+ list(quat), 1) + [3.0]
        arm.cart_movearm('l', [goal], "tetris", True)

        raw_input("next pose")



def distance_between_two_pts( p1,p2):
    pts = zip(p1,p2)
    dx = [(x1 - x2)**2 for (x1,x2) in pts]
    return sum(dx)

def dist(p1,p2):
    return sum([ (x1-x2)**2 for x1 in p1 for x2 in p2  ]  )

def timing(traj):
    cur_pose = arm.get_cartesian_pose('tetris')['l']
    p0 = cur_pose[:6]
    p1 = traj[0][:6]
    time = 0.3*dist(p0,p1)
    time_gain =1. #XXX add extra 0
    p0=p0[:3]
    for i in range(len(traj)):
        p1 = traj[i][:3]
        time += max(time_gain*dist(p0,p1), min_time)
        p0 = p1
        traj[i] += [time]
    return traj


    

def new_do_tetris_prim(arm,prim):
    (px,py), direction, final = prim

    if direction == "down":
        quat = horz
        final_x = px
        final_y = 0

    elif direction == "up": 
        quat = horz
        final_x = px
        final_y = H

    elif direction == "right":
        quat = vert
        final_x = W
        final_y = py

    elif direction == "left":
        quat = vert
        final_x = 0
        final_y = py
    
    if final != None:
        final_x, final_y = final

    x_traj =  np.linspace(px*block_size, final_x*block_size, interpolate)
    y_traj =  np.linspace(py*block_size, final_y*block_size, interpolate)
    
    xy_traj= zip(x_traj, y_traj,  [z_down]*interpolate)
    xy_traj = [list(a) for a in xy_traj]
    
    # above poses
    start_up  = list(xy_traj[0])
    start_up[2] += z_above  # start above
    start_down = list(xy_traj[0]) # stiff pose to table
    
    start_traj = [
                    stiff_pose(start_up  + quat,1. )  , 
                    stiff_pose(start_down  + quat,1. )  ]
    xy_traj = [ push_down(p+quat) for p in xy_traj[1:]]
    trajectory = start_traj + xy_traj
    timing(trajectory) # update with timing

    arm.cart_movearm('l', trajectory, "tetris", True)
    arm.cart_movearm('l', release_traj(arm), 'tetris', True)

def test_tetris(arm):
    T = get_table( arm, "tetris")
    # 10 x ~20 blocks get_tetris_traj(arm, ppm, px,py, direction)  a block is about 13 mm

    

    prims= [
             # push first block into corner
            ( (5,25), "down", None),   # push down from top middle (5, 12)
            ( (2,4), "right", None),  # push from bottom left corner to right

            # push second block next to first block (repeat 1st sequence)
            ( (5,25), "down", None),
            ( (2,4), "right", None),

            # push third block on top of first block 
            ( (5,25), "down", (5,10) ),  # only push halfway
            ( (2,10), "right", None ),  # push block to side
            ( (7,20), "down", None ) # push down on first block

            ]

    pose =[ 7*block_size, 20*block_size, 0]  + rot_r

    first = stiff_pose(pose, 1) + [5]
    special = right_slide_down(pose) + [20]

    arm.cart_movearm('l', [first, special], "tetris", True)
    raw_input()
    for p in prims:
        print new_do_tetris_prim(arm, p)

 

""" TODO
    bimanual
    
    thing block = (.5, .5, .5, .5)
    delightful: compute time based on distance

    seperate create tetris prim thing it's ugly

    delightful: pick up sticks!

    next: localize in corner routine
    next: use thin left arm

    next : open cv projection

"""


def test_hover_corners(arm):
    for i in range(10):
        T = get_table(arm)
        # hover over tetris board
        z = 100
        for (x,y) in tetris:
            pos,_ = point_transformed(T, [1e-3*p for p in [x,y,z]])
            goal = stiff_pose( pos+ list(vert) ) + [3.0]
            arm.cart_movearm('l', [goal], "base_link", True)



def initialize_tetris(arm, reset_all, set_gripper):
    arm.command_torso(0.26)
    move_arms_to_side(arm)

    if reset_all:
        arm.command_torso(0.3)
        if set_gripper:
            arm.open_gripper('r')
            arm.open_gripper('l')
            raw_input("close right gripper")
            arm.close_gripper('r')
            raw_input("close left gripper")
            arm.close_gripper('l')
         
        #BT = TableBroadcaster(arm.tf_listener)
    arm.close_gripper('r')
    arm.close_gripper('l')

 
 
if __name__=="__main__":
    
    
    parser = ArgumentParser(description="hello ari")
    parser.add_argument("--init", action="store_true")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--gripper", action="store_true")
    args = parser.parse_args()

    if args.init or args.reset:
        if args.reset:
            rospy.init_node('table')
        else:
            rospy.init_node("init_tetris")
        arm = ArmController()
        initialize_tetris(arm, args.reset, args.gripper)
    else:
        
        rospy.init_node("tetris")
        arm = ArmController()
        #initialize_tetris(arm, args.reset, args.gripper)
        #test_hover2(arm)
        test_tetris(arm)


    """#arm.joint_movearms(init_pose)
    arm.close_gripper('r')
    arm.close_gripper('l')
    arm.command_torso(0.3)
    do_tetris_prim( arm, T_inv, middle )
    test_tetris(arm)"""

