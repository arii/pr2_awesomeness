#!/usr/bin/env python
# Author: Ariel Anders

import roslib
roslib.load_manifest('pr2_pick_and_place_tutorial')
import rospy
import actionlib
from object_manipulation_msgs.msg import PickupAction, PickupGoal, PickupResult
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import ManipulationResult, GraspableObject
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import \
    TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
from geometry_msgs.msg import Vector3Stamped
from object_manipulator.convert_functions import *


GOAL_STATE = 3
OBJECT_DETECTION_SERVICE_NAME = "/object_detection"
COLLISION_PROCESSING_SERVICE_NAME = \
    "/tabletop_collision_map_processing/tabletop_collision_map_processing"
PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup"



class ArghGrasp:
    def __init__(self):

        #create service and action clients
        self.object_detection_srv = rospy.ServiceProxy(
            OBJECT_DETECTION_SERVICE_NAME,
            TabletopDetection)

        self.collision_processing_srv = rospy.ServiceProxy(
            COLLISION_PROCESSING_SERVICE_NAME,
            TabletopCollisionMapProcessing)

        self.pickup_client = actionlib.SimpleActionClient(
            PICKUP_ACTION_NAME,
            PickupAction)

        # wait for detection client
        rospy.loginfo("Waiting for object detection service to come up")
        rospy.wait_for_service(OBJECT_DETECTION_SERVICE_NAME)

        rospy.loginfo("Waiting for collision processing service to come up");
        rospy.wait_for_service(COLLISION_PROCESSING_SERVICE_NAME)

        # wait for pickup client
        rospy.loginfo("Waiting for action client " + PICKUP_ACTION_NAME)
        self.pickup_client.wait_for_server()
        

    def pick_up(self, whicharm="l", NUM_TRIES=5, near_pose=None):
        self.pickup_client.cancel_goal()

        for i in range(NUM_TRIES):
          rospy.loginfo("Attempt %d  to grasp " % i )
          # call the tabletop detection
          rospy.loginfo("Calling tabletop detector")

          detection_call = TabletopDetectionRequest()

          # we want recognized database objects returned
          # set this to false if you are using the pipeline without the database
          detection_call.return_clusters = True

          #we want the individual object point clouds returned as well
          detection_call.return_models = True
          detection_call.num_models = 1


          detection_call_response = self.object_detection_srv(detection_call).detection
          if not detection_call_response:
              rospy.loginfo("Tabletop detection service failed")
              success=False; break

          if detection_call_response.result != detection_call_response.SUCCESS:
              rospy.loginfo("Tabletop detection returned error code %d", 
                        detection_call_response.result)
              success=False; break
            
          if len(detection_call_response.clusters)==0\
            and len(detection_call_response.models)==0 :
              rospy.loginfo("The tabletop detector detected the table, "
                        "but found no objects")
              success=False; break



          # call collision map processing
          rospy.loginfo("Calling collision map processing")

          processing_call = TabletopCollisionMapProcessingRequest()

          # pass the result of the tabletop detection 
          processing_call.detection_result = detection_call_response
          print detection_call_response.models
          import pdb;pdb.set_trace()

          # ask for the existing map and collision models to be reset
          processing_call.reset_collision_models = True
          processing_call.reset_attached_models = True

          #ask for the results to be returned in base link frame
          processing_call.desired_frame = "base_link"

          processing_call_response = self.collision_processing_srv(processing_call)
          if not processing_call_response:
              rospy.loginfo("Collision map processing service failed")
              success=False; break

          #the collision map processor returns instances of graspable objects
          if len(processing_call_response.graspable_objects) == 0:
              rospy.loginfo("Collision map processing returned no graspable objects")
              success=False; break
          graspable_objects = processing_call_response.graspable_objects

          # sort graspable objects
          import pdb;
          # sort graspable objects
          objects_with_offsets = [None]*len(graspable_objects)
          #near_point = np.array(list(near_pose[:3]) + [1.0])
          for i, obj in enumerate(graspable_objects):
            pdb.set_trace()
            points  = point_cloud_to_mat(obj.cluster)
            #centroid =np.reshape(np.mean(points, axis=1), 4) - near_point
            #offset = np.linalg.norm(centroid - near_point)
            #objects_with_offsets[i] = (offset,obj)
          #ordering = sorted(objects_with_offsets, key=lambda item:float((item[0])))
          #graspable_objects = [items[1] for items in ordering]

          #print [items[0] for items in ordering]
          for i, grasp_object in enumerate(graspable_objects):
              
              # call object pickup
              rospy.loginfo("Calling the pickup action");
              pickup_goal = PickupGoal()

              # pass one of the graspable objects returned 
              # by the collision map processor
              #pickup_goal.target = processing_call_response.graspable_objects[0]
              pickup_goal.target = grasp_object

              # pass the name that the object has in the collision environment
              # this name was also returned by the collision map processor
              pickup_goal.collision_object_name = \
                  processing_call_response.collision_object_names[i]

              # pass the collision name of the table, also returned by the collision 
              # map processor
              pickup_goal.collision_support_surface_name = \
                  processing_call_response.collision_support_surface_name

              # pick up the object with the left arm
              if whicharm=="l":
                  pickup_goal.arm_name = "left_arm"
              else:
                  pickup_goal.arm_name = "right_arm"

              # we will be lifting the object along the "vertical" direction
              # which is along the z axis in the base_link frame

              direction = Vector3Stamped()
              direction.header.stamp = rospy.Time.now()
              direction.header.frame_id = "base_link"
              direction.vector.x = 0
              direction.vector.y = 0
              direction.vector.z = 1
              pickup_goal.lift.direction = direction
              #request a vertical lift of 10cm after grasping the object

              pickup_goal.lift.desired_distance = 0.1;
              pickup_goal.lift.min_distance = 0.05;
              #do not use tactile-based grasping or tactile-based lift

              pickup_goal.use_reactive_lift = False;
              pickup_goal.use_reactive_execution = False;
              
              #pickup_goal.allow_gripper_support_collision = True
              #pickup_goal.ignore_collisions = True
              #send the goal
              rospy.loginfo("Waiting for the pickup action...")
              self.pickup_client.send_goal_and_wait(pickup_goal,rospy.Duration(15))
              
              pickup_result =self.pickup_client.get_result()
              assert isinstance(pickup_result, PickupResult)
              success = pickup_result.manipulation_result.value ==\
                pickup_result.manipulation_result.SUCCESS
               
              if success:
                  rospy.loginfo("Success! Grasped Object.")
                  return success
        if not success:
            rospy.loginfo("failed to grasp object")
        return success
if __name__=="__main__":
    rospy.init_node("pick_and_place_app")
    p = ArghGrasp()
    p.pick_up()
    

