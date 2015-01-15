#!/usr/bin/env python
# Author: Ariel Anders

import roslib; roslib.load_manifest('pr2_awesomeness')
import rospy
from pr2_mechanism_msgs.srv import SwitchController, ListControllers
from utils import wait_for_service
" This allows you switch between different arm controllers"

class Pr2ControllerManager:

    def __init__(self, use_ee_cart = True):
        switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
        list_controllers_serv_name = 'pr2_controller_manager/list_controllers'
        wait_for_service(switch_controller_serv_name)
        wait_for_service(list_controllers_serv_name)
        
        self.switch_controller_service = \
            rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
        self.list_controllers_service = \
            rospy.ServiceProxy(list_controllers_serv_name, ListControllers)
        
        self.controllers = {
                'joint': "_arm_controller",
                'cartesian': "_cart"
                }
        if use_ee_cart:
            self.controllers['cartesian']= "_arm_cart_imped_controller"
                


    def get_controllers_status(self, whicharm):
        resp = self.list_controllers_service()
        controllers = dict(zip(resp.controllers, resp.state))
        joint_name = whicharm + self.controllers["joint"]
        cart_name = whicharm + self.controllers["cartesian"]

        joint_loaded =  joint_name in controllers
        cart_loaded = cart_name in controllers
        if not cart_loaded:
            rospy.loginfo("Cartesian controller not loaded! error")
        joint_running = controllers[joint_name] =="running" \
            if joint_loaded else False
        cart_running = controllers[cart_name] =="running" \
            if cart_loaded else False
        rospy.loginfo("joint controller running: %s, cart controller running: %s "\
            %(joint_running, cart_running))
        
        return joint_running, cart_running


    def start_controller(self, whicharm, start_ctrl):
        if not start_ctrl in ["cartesian", "joint"]:
                rospy.loginfo("start controller is cartesian or joint")
        stop_ctrl = "cartesian" if start_ctrl == "joint" else "joint"
        start_name = whicharm + self.controllers[start_ctrl]
        stop_name = whicharm + self.controllers[stop_ctrl]


        # Get current controllers status
        resp = self.list_controllers_service()
        controllers_status = dict(zip(resp.controllers, resp.state))

        # are controllers loaded?
        stop_loaded =  stop_name in controllers_status
        start_loaded =  start_name in controllers_status

        # are controllers running?
        stop_running = controllers_status[stop_name]=="running"\
                if stop_loaded else False  
        start_running = controllers_status[start_name]=="running"\
                if start_loaded else False  
        
        rospy.loginfo(("starting %s, stoppings %s for arm %s. Current status "\
                "of start, stop: %s, %s " % (start_ctrl, stop_ctrl, whicharm,\
                start_running, stop_running)))

        # turn start controller on and stop controller off
        if start_running and stop_running:
            self.switch_controller_service([], [stop_name],2)
        elif not start_running and not stop_running:
            self.switch_controller_service([start_name], [],2)
        elif not start_running and stop_running:
            self.switch_controller_service([start_name], [stop_name],2)
        else:
            # don't have to switch anything.  correct config
            pass
if __name__=="__main__":
    pr2_controller_manager = Pr2ControllerManager()
    for arm in ['l', 'r']:
        print "testing arm %s " % arm
        raw_input(pr2_controller_manager.get_controllers_status(arm))
        for ctrl in ["cartesian", "joint"] :
            print "starting %s " % ctrl
            pr2_controller_manager.start_controller(arm, ctrl)
            raw_input("status after starting %s is joint,cart = %s" % \
                    (ctrl, pr2_controller_manager.get_controllers_status(arm)))

