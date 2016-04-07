import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import ee_cart_imped_action
from pr2_controller_manager import Pr2ControllerManager
from pick_and_place_manager import PickAndPlaceManager
from object_manipulator.convert_functions import lists_to_pose_stamped, get_transform, mat_to_pos_and_quat
import numpy as np
from quack import Arm
from tf_transformer import TfTransformer
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf

def main():
    print "hello!"
    pr2_controller_manager = Pr2ControllerManager('r')
    pr2_controller_manager.start_controller('r', "cartesian")

    control = ee_cart_imped_action.EECartImpedClient('right_arm')
    control.addTrajectoryPoint(0.5, 0, 0, 0, 0, 0, 1,
            1000, 1000, 1000, 30, 30, 30, 
            False, False, False, False, False,False, 
            4, '/torso_lift_link');
    control.addTrajectoryPoint(0.85, 0, 0 ,0, 0, 0, 1,
            1000, 1000, 1000, 30, 30, 30,
            False, False, False, False, False,
            False, 6, '/torso_lift_link');
    control.sendGoal()

def stiff_pose(pose):
    return list(pose) + [800]*3 + [30]*3 + [False]*6

def push_down(pose):
    
    fx,fy,fz = [600]*3  # max stiffness
    ox,oy,oz = [30]*3
    fz = -3  # force in downard direction
    states = [False]*6
    states[2] = True # use force control for Z

    return list(pose) + [fx,fy,fz,ox,oy,oz] + states 


def test_quat():
    sqr2 = np.sqrt(2.0)/2.0
    sqr7 = np.sqrt(7.0)/4.0

    vert =[  (0,sqr2, 0, sqr2) , (sqr2, 0, -sqr2, 0) ]
    horz =[ (0.5, 0.5, -0.5, 0.5), (-0.5, 0.5, 0.5, 0.5) ]

    rot_r=[ (-sqr7, 0.25, sqr7, 0.25) , (0.25, sqr7, -.25, sqr7) ]
    rot_l=[ (-.25, sqr7,0.25, sqr7), (-sqr7, -0.25, sqr7, -0.25) ]
    return vert, horz, rot_r, rot_l

def move_quat(arm, ppm):
    types = ["vert", "horz", "rot_r", "rot_l"]
    whicharm = 1 # left
    x,y,z,oa,ox,oy,oz = ppm.return_current_pose_as_list(whicharm)
    pos = (x,y,z)

    for i,pose_type in enumerate(test_quat()):
        print("test %s" % types[i])
        for j in range(2):

            raw_input("test %s-- %d" % (types[i], j))
            goal = stiff_pose(pos + pose_type[j]) + [2.0]
            print goal

            arm.cart_movearm('l', [goal], "base_link", True)


tetris = [(180, 80), (180,332), (28,332), (28,80)]
sqr2 = np.sqrt(2.0)/2.0
sqr7 = np.sqrt(7.0)/4.0
vert =  (0,sqr2, 0, sqr2) 
horz =  (0.5, 0.5, -0.5, 0.5)
rot_r=  (-sqr7, 0.25, sqr7, 0.25)
rot_l=  (-.25, sqr7,0.25, sqr7)
blank_T = np.matrix(tf.transformations.quaternion_matrix( (0,0,0,1)))


def get_table(ppm,arm):

    ppm.follow_corners()
    tf_transformer = TfTransformer(arm.tf_listener)
    T_base_table =  get_transform(arm.tf_listener, "base_link", "table").getI()
    return T_base_table


def point_transformed( T, pos):
    offset = blank_T
    offset[0:3,3] = np.matrix(pos).T
    new_pose = T * offset
    trans, rot = mat_to_pos_and_quat(new_pose)
    return trans,rot
     
def test_hover_corners(arm, ppm):
    for i in range(10):
        T = get_table(ppm,arm)
        
        # hover over tetris board
        z = 100
        for (x,y) in tetris:
            pos,_ = point_transformed(T, [1e-3*p for p in [x,y,z]])
            goal = stiff_pose( pos+ list(vert) ) + [3.0]
            arm.cart_movearm('l', [goal], "base_link", True)

def prim_test(arm, ppm):
    T = get_table(ppm, arm)
    z_above = 100
    z_down =20# 100
    poses = [
            [ (100,360, z_above), horz, False ], # (x,y, z), quat, pushdown
            [ (100,360, z_down), horz, False ], 
            [ (100,360, z_down), horz, True ], # (push block down)

            [ (100,80, z_down), horz, True],
            [ (90,140, z_down), horz, True], # gentle release

            [ (90,140, z_above), horz, False], #pull up rotate
            [ (80,140, z_above), vert, False],

            [ (80,140, z_down), vert, False], # place down far away
            [ (180,110, z_down), vert, True], # localize

            [ (70,110, z_down), vert, True], # push to side
            [ (70,110, z_above), vert, False],
            ]
    send_poses = []
    sum_time = 0
    for i,((x,y,z), quat, pushDown) in enumerate(poses):
        pos,_ = point_transformed(T, [1e-3*p for p in [x,y,z]])
        goal = pos + list(quat)
        if pushDown:
            goal = push_down(goal)
        else:
            goal = stiff_pose(goal)
        if i == 0:
            time = 3
        else:
            if sum( [abs(pos[i] - prev_pos[i])  for i in range(3)] ) <= 80:
                time = 1
            else:
                time = 2
        sum_time  += time
        send_poses.append(goal + [sum_time] )

        prev_pos  = pos
        #send_poses.append(goal + [2.0*i + 5.0] )
    print send_poses
    arm.cart_movearm('l', send_poses, "base_link", True)



if __name__=="__main__":
    rospy.init_node('hello_ari')
    ppm = PickAndPlaceManager()
    arm = Arm(ppm.tf_listener)
    rospy.loginfo("getting pose")

    arm.command_gripper('r', 0)
    arm.command_gripper('l', 0)
    #test_hover_corners(arm,ppm)
    for i in range(5):
        prim_test(arm,ppm)

