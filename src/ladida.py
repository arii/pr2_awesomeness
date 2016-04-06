import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import ee_cart_imped_action
from pr2_controller_manager import Pr2ControllerManager
from pick_and_place_manager import PickAndPlaceManager
import numpy as np
from quack import Arm
from tf_transformer import TfTransformer

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



tetris = [(158, 80), (158,253), (28,332), (28,80)]
sqr2 = np.sqrt(2.0)/2.0
sqr7 = np.sqrt(7.0)/4.0
vert =  (0,sqr2, 0, sqr2) 
horz =  (0.5, 0.5, -0.5, 0.5)
rot_r=  (-sqr7, 0.25, sqr7, 0.25)
rot_l=  (-.25, sqr7,0.25, sqr7)



if __name__=="__main__":
    rospy.init_node('hello_ari')
    ppm = PickAndPlaceManager()
    arm = Arm(ppm.tf_listener)
    rospy.loginfo("getting pose")

    arm.command_gripper('r', 0)
    arm.command_gripper('l', 0)


    
    pos,quat = ppm.follow_corners()
    #move_quat(arm, ppm)
    tf_transformer = TfTransformer(arm.tf_listener)
    trans,rot =  tf_transformer.get_transform("base_link", "table")
    
    trans,rot = (0,0,0), (0,0,0,1) 
    offset = np.matrix(tf.transformations.quaternion_matrix(rot))
    
    # hover over tetris board
    
    z = 0.1
    for (x,y) in tetris:
        pose = (x,y,z)
        offset [0:3,3] = np.matrix(trans).T
        mat = np.matrix(tf.transformations.quaternion_matrix(rot))
            mat[0:3,3] =  np.matrix(trans).T
            new_pose = mat*self.offset

