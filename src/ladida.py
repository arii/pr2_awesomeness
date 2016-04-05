import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
import ee_cart_imped_action
from pr2_controller_manager import Pr2ControllerManager
from pick_and_place_manager import PickAndPlaceManager
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



if __name__=="__main__":
    rospy.init_node('hello_ari')
    ppm = PickAndPlaceManager()
    pos,quat = ppm.follow_corners()

    bottom_left = (80, 28)
    top_left = (253, 80)
    bottom_right = (80, 
    


    raw_input()

    #main()

