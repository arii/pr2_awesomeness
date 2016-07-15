#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_awesomeness')
import rospy
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.msg import TabletopDetectionResult
import tf
import scipy
import threading
from object_manipulator.convert_functions import change_pose_stamped_frame, pose_to_mat
import sys
from vision_stuff import Echo
from std_msgs.msg import Bool 
##Manager for pick and place actions
class TableBroadcaster():


    def __init__(self, tf_listener=None):
        self.lock = threading.Lock()

        self.stereo_camera_frame = rospy.get_param("~stereo_camera_frame", "/head_mount_kinect_rgb_optical_frame")
        self.freeze_update = False

        self.freeze_sub = rospy.Subscriber('freeze', Bool, self.toggleFreeze)

        #service names
        self.grasper_detect_name = 'object_detection'

        if tf_listener==None:
            #start tf listener
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.tf_broadcaster =  tf.TransformBroadcaster()
        self.rate = rospy.Rate(0.25)
      
        self.image_detection = Echo(self.tf_listener,table=False)# table=True)
        #service proxies
        self.grasper_detect_srv = rospy.ServiceProxy(self.grasper_detect_name, TabletopDetection)     

        self.tabletop_detection_result_dict = {}
        for element in dir(TabletopDetectionResult):
            if element[0].isupper():
                self.tabletop_detection_result_dict[eval('TabletopDetectionResult.'+element)] = element
        self.pos = None
        self.thread = threading.Thread(target=self.broadcast_table)
        self.thread.start()
        self.start_broadcast_table()

    def toggleFreeze(self, msg):
        self.freeze_update = msg.data
        rospy.loginfo("Freeze update: %s " % msg.data)



    
    def broadcast_table(self):
        quat = (0,0,-(2**.5),2**.5)
        while not rospy.is_shutdown():
            # tetris
            if self.pos == None:
                self.rate.sleep()
            else:
                pos = tuple(self.pos)
                (x,y,z) = pos
                self.tf_broadcaster.sendTransform(pos, quat, rospy.Time.now(), "table", "base_link")
                x  += .065
                y -= 0.01
                #x  += 0.095
                #y -= 0.013
                z+= .02
                pos = (x,y,z)
                self.tf_broadcaster.sendTransform(pos, quat, rospy.Time.now(), "tetris", "base_link")


    def start_broadcast_table(self):
        while not rospy.is_shutdown():

            self.rate.sleep()
            if not self.freeze_update:
                corners = self.find_table()

                if corners != None:

                    x= corners[0,:].min()
                    y= corners[1,:].max()
                    z= corners[2,:].max()
                    #if self.image_detection.table_pos !=None:
                        #xb,yb,_ = self.image_detection.table_pos
                        #x= x*0.5 + xb*0.5
                        #y= y*0.5 + yb*0.5
                    self.pos = (x,y,z)
                    """
                    # tetris
                    pos = (x,y,z)
                    quat = (0,0,-(2**.5),2**.5)
                    self.tf_broadcaster.sendTransform(pos, quat, rospy.Time.now(), "table", "base_link")
                    x  += .08
                    y -= 0.03
                    pos = (x,y,z)
                    quat = (0,0,-(2**.5),2**.5)
                    reset = rospy.Time.now() + rospy.Duration(0.5)
                    while not rospy.is_shutdown() and ( rospy.Time.now() < reset) :
                        self.tf_broadcaster.sendTransform(pos, quat, rospy.Time.now(), "tetris", "base_link")
                        #rospy.sleep(0.05)
                    """

         
    ##call tabletop object detection and collision_map_processing 
    #(detects table/objects and adds them to collision map)
    def find_table(self): 
        rospy.loginfo("calling tabletop detection")

        det_req = TabletopDetectionRequest()
        det_req.return_clusters = 0
        det_req.return_models = 0
        det_req.num_models = 0
        self.detected_table = None
        #call tabletop detection, get a detection result
        try:
            det_res = self.grasper_detect_srv(det_req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.grasper_detect_name, e))
            rospy.signal_shutdown("no table top detection service")
            return None

        if det_res.detection.result == det_res.detection.SUCCESS:
            rospy.loginfo("tabletop detection reports success")
            self.detected_table = det_res.detection.table
            return self.update_table_info()
        else:
            rospy.logerr("tabletop detection failed with error %s, trying again"%\
                self.tabletop_detection_result_dict[det_res.detection.result])
         
            return None


    def update_table_info(self):
        #find the table's front edge and height
        base_link_pose_stamped = change_pose_stamped_frame(self.tf_listener, self.detected_table.pose, 'base_link')
        table_mat = pose_to_mat(base_link_pose_stamped.pose)
        corners = scipy.matrix([[self.detected_table.x_min, self.detected_table.y_min,0,1],
                                [self.detected_table.x_min, self.detected_table.y_max,0,1],
                                [self.detected_table.x_max, self.detected_table.y_min,0,1],
        #num_models
                                [self.detected_table.x_max, self.detected_table.y_max,0,1]]).T
        transformed_corners = table_mat * corners
        front_edge_x = transformed_corners[0,:].min()
        height = transformed_corners[2,:].max()
        self.table_front_edge_x = front_edge_x
        self.table_height = height
        rospy.loginfo("table front edge x: %5.3f"%front_edge_x)
        rospy.loginfo("table height: %5.3f"%height)
       
        return transformed_corners
 
if __name__ == '__main__':

    rospy.init_node('table_publisher')
    tb = TableBroadcaster()
    #tb.broadcast_table()
    rospy.spin() 
