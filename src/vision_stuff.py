import rospy
import roslib; roslib.load_manifest('pr2_awesomeness')
import cv2 #2.4.2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
import tf
import image_geometry
from geometry_msgs.msg import Point, PointStamped
from object_manipulator.convert_functions import get_transform




class Echo:
    def __init__(self, tf_listener=None):
        self.node_name = "Echo"
        self.camera_info = None
        self.camera = image_geometry.PinholeCameraModel()
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.thread_lock = threading.Lock()
        self.frames = ["tetris", "l_gripper_tool_frame"]

        self.sub_image = rospy.Subscriber("/head_mount_kinect/rgb/image_color",\
        Image, self.cbImage)
        self.sub_cam_info = rospy.Subscriber("/head_mount_kinect/rgb/camera_info",\
        CameraInfo, self.cbCameraInfo)

        self.pub_image = rospy.Publisher("~echo_image", Image)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbCameraInfo(self, msg):
        self.camera_info = msg
        self.camera.fromCameraInfo(msg)


    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if self.camera_info == None: return
        if not self.thread_lock.acquire(False):
            return
        image_cv = np.array(self.bridge.imgmsg_to_cv(image_msg)[:,:] )

        image = cv2.cv.fromarray(image_cv)

        
        for frame in self.frames:
            camera_frame = self.camera_info.header.frame_id
            time = rospy.Time.now()
            #import pdb; pdb.set_trace()
            transform = None
            try:
                t = self.tf_listener.getLatestCommonTime(camera_frame, frame)
                transform = self.tf_listener.lookupTransform(camera_frame, frame, t)

            except:
                print "could not get transform"
                #rospy.sleep(0.5)
            
            if transform != None:
                x,y =( int(p) for p in  self.camera.project3dToPixel(transform[0]))
                cv2.circle(image_cv, (x,y) , 5, (255,0,0), -1)
        
        cv2.circle(image_cv, (100,200) , 5, (0,255,0), -1)
        p =  self.camera.projectPixelTo3dRay( (100,200))

        print p
        T = get_transform(self.tf_listener, camera_frame, "base_link")
        print T

        v = np.matrix(list(p) + [1.0])
        #print v

        print T*v.T  # <-- this shows the position (200,200) converted to base link!!!

        try:
            self.pub_image.publish(\
            self.bridge.cv_to_imgmsg(image, "rgb8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

