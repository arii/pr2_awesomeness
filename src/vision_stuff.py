import rospy
import roslib; roslib.load_manifest('pr2_awesomeness')
import cv2 #2.4.2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
import tf
import image_geometry
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, Point, Quaternion
from object_manipulator.convert_functions import get_transform

class Detector:
    RED1 = [np.array(x, np.uint8) for x in [[0,140,100], [15, 255,255]] ]
    RED2 = [np.array(x, np.uint8) for x in [[165,140,100], [180, 255, 255]] ]
    YELLOW = [np.array(x, np.uint8) for x in [[25,100,150], [35, 255, 255]] ] 
    ORANGE = [np.array(x, np.uint8) for x in [[0,80,80], [22, 255,255]] ]
    GREEN = [np.array(x, np.uint8) for x in [[25,80,100], [60, 255,255]] ]
    BLUE = [np.array(x, np.uint8) for x in [[110, 50,50], [120,255,255 ]] ]
    colors={'RED1':RED1, 'RED2':RED2, 'YELLOW':YELLOW, 'ORANGE':ORANGE, 'GREEN':GREEN, 'BLUE': BLUE}

    def get_filtered_contours(self,img, contour_type):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        if contour_type == "CANNY":
            """
            frame_threshed = cv2.inRange(hsv_img, self.DUCK[0], self.DUCK[1])
            frame_threshed = cv2.adaptiveThreshold(frame_threshed,255,\
                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)"""
            thresh = cv2.Canny(hsv_img, 50,100)
        else:

            color_val = self.colors[contour_type]
            frame_threshed = cv2.inRange(hsv_img, color_val[0], color_val[1] )
            ret,thresh = cv2.threshold(frame_threshed,50,255,0)
        
        filtered_contours = []
        
        contours, hierarchy = cv2.findContours(\
                thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])
        height,width = img.shape[:2]
        for (area,(cnt)) in contour_area:
        # plot box around contour
            x,y,w,h = cv2.boundingRect(cnt)
            box = (x,y,w,h)
            #import pdb; pdb.set_trace()
            if not(area >100 ):
                    continue
            mask = np.zeros(thresh.shape,np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)
            mean_val = cv2.mean(img,mask = mask)
            aspect_ratio = float(w)/h
            filtered_contours.append( (area, (cnt, box, aspect_ratio, mean_val, area)) )
        return filtered_contours


    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        """
        height,width = img.shape[:2]

        # get filtered contours
        #red1 = self.get_filtered_contours(img, "RED1")
        #red2 = self.get_filtered_contours(img, "RED2")
        all_contours = self.get_filtered_contours(img, "GREEN")
        #yellow = self.get_filtered_contours(img, "YELLOW")
        
        #all_contours = red1 + red2 + orange
        all_contours= sorted(all_contours,reverse=True, key=lambda x: x[0])
        center = None

        for area, (cnt, box,  aspect_ratio, mean_color, area) in  all_contours:
            # plot box around contour
            x,y,w,h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,"ari", (x,y), font, 0.5,mean_color,4)
            cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)
            center, (width,height), rotation = cv2.minAreaRect(cnt)
        return img, center
        """
        return self.tetris_bounds(img)


    def tetris_bounds(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''


        height,width = img.shape[:2]

        green_contours = self.get_filtered_contours(img, "GREEN")
        if len(green_contours) < 2: return img, None, None

        green_contours= sorted(green_contours,reverse=True, key=lambda x: x[0])
        center = table_corner=rotation=None
        avg_rotation = 0

        
        for area, (cnt, box,  aspect_ratio, mean_color, area) in  green_contours[:2]:
            # plot box around contour
            x,y,w,h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            #cv2.putText(img,"table", (x,y), font, 0.5,mean_color,4)
            cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,1)
            result = cv2.minAreaRect(cnt)
            center, (width,height), rotation = result
            avg_rotation += rotation*.5
            if y > 250: # assume bottom
                for px, py in cv2.cv.BoxPoints(result):
                    if px < center[0] and py > center[1]:
                        table_corner = int(px), int(py)
                #table_corner = tuple(int(p) for p in lower_left)

        if table_corner == None:
            print "move table closer"
        else:
            print "table corner is located at %s with rotation %s " % (table_corner, rotation)
        return img, table_corner, rotation


    def find_objects(self, img):
        
        all_contours = self.get_filtered_contours(img, "BLUE")
        height,width = img.shape[:2]

        all_contours= sorted(all_contours,reverse=True, key=lambda x: x[0])
        center = None
        results = []
        for area, (cnt, box,  aspect_ratio, mean_color, area) in  all_contours:
            # plot box around contour
            #R,G, B, a = mean_color
            #if B < 1.2*G: continue

            x,y,w,h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,"ari", (x,y), font, 0.5,mean_color,4)
            cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)
            center, (width,height), rotation = cv2.minAreaRect(cnt)
            results.append(cv2.minAreaRect(cnt))


        return img, results

class Echo:
    def __init__(self, tf_listener=None, table=False):
        self.node_name = "Echo"
        self.camera_info = None
        self.camera = image_geometry.PinholeCameraModel()
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        self.detector = Detector()
        self.table_pos = None
        self.table= table
        self.thread_lock = threading.Lock()
        self.frames = ["tetris", "l_gripper_tool_frame", "table"]

        self.sub_image = rospy.Subscriber("/head_mount_kinect/rgb/image_color",\
        Image, self.cbImage)
        self.sub_cam_info = rospy.Subscriber("/head_mount_kinect/rgb/camera_info",\
        CameraInfo, self.cbCameraInfo)

        self.pub_image = rospy.Publisher("~echo_image", Image)
        self.pub_poses = rospy.Publisher("~object_poses", PoseArray)
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
        if self.table:
            image_cv = self.broadcast_table(image_cv)
        #else:
        image_cv = self.broadcast_objects(image_cv)
        #image_cv = self.label_frames(image_cv)

        image = cv2.cv.fromarray(image_cv)
        try:
            self.pub_image.publish(\
            self.bridge.cv_to_imgmsg(image, "rgb8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()



    def label_frames(self, image_cv): 
        for frame in self.frames:
            camera_frame = self.camera_info.header.frame_id
            time = rospy.Time.now()
            transform = None
            try:
                t = self.tf_listener.getLatestCommonTime(camera_frame, frame)
                transform = self.tf_listener.lookupTransform(camera_frame, frame, t)

            except:
                print "could not get transform"
                #rospy.sleep(0.5)

            if transform != None:
                x,y =( int(p) for p in  self.camera.project3dToPixel(transform[0]))
                cv2.circle(image_cv, (x,y) , 5, (255,0,0), 1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image_cv,frame,(x,y), font, 1,(255,255,255),2)

        
        return image_cv
       
    def pixel_to_base_link(self,image, ( x,y)):
        cv2.circle(image, (int(x),int(y)), 1, (255,0,0),  1 )

        p =  self.camera.projectPixelTo3dRay( (x,y))
        T = get_transform(self.tf_listener, self.camera_info.header.frame_id, "base_link")

        v = np.matrix(list(p) + [1.0])
        pt =  T*v.T  # <-- this shows the position (200,200) converted to base link!!!
        return [float(p) for p in list(pt)[:3]]

    def broadcast_table(self, image_cv):
        image_cv, table_corner, rotation  = self.detector.tetris_bounds(image_cv)
        if rotation  < -2: rospy.loginfo("table is rotated %s !!"% rotation)
        if table_corner != None:
            self.table_pos =  self.pixel_to_base_link(image_cv, table_corner)
            x,y = table_corner
            cv2.circle(image_cv, (x,y) , 5, (255,0,0), 1)
        else:
            self.table_pos = None
        return image_cv

    def broadcast_objects(self, image_cv):
        image_cv, results  = self.detector.find_objects(image_cv)
        poses = []
        for result in results:
            center, (width,height), rotation  = result
            pos = Point(*self.pixel_to_base_link(image_cv, center))
            #q = Quaternion(x=rotation) # placeholder
            poses.append( Pose(pos, Quaternion(z=1.0)) ) # placeholder)
        stamped_poses = PoseArray()
        stamped_poses.header.frame_id = "base_link"
        stamped_poses.header.stamp  = rospy.Time(0)
        stamped_poses.poses = poses
        
        self.pub_poses.publish(stamped_poses)


        return image_cv



if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo(None, False)
    rospy.spin()

