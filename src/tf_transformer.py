#!/usr/bin/env python
# author: Ariel Anders
import numpy as np
import rospy
import roslib; 
roslib.load_manifest("pr2_awesomeness")
import tf
from object_manipulator.convert_functions import mat_to_pos_and_quat


class TfTransformer:
    def __init__ (self, tf_listener = None):
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        trans,rot = (0.17859586467670954, 0.022878946330841865, -0.0070694575988708985), (-0.0635704601963138, 0.05770569995935729, -0.7367079894246215, 0.6707385385532587)

        self.offset = np.matrix(tf.transformations.quaternion_matrix(rot))
        self.offset [0:3,3] = np.matrix(trans).T
    
    def get_transform(self, tool_frame, base_frame):
        try:
            self.tf_listener.waitForTransform(\
                tool_frame, base_frame, rospy.Time(0), rospy.Duration(2))
            (trans,rot) = self.tf_listener.lookupTransform(\
                tool_frame, base_frame,rospy.Time())
            mat = np.matrix(tf.transformations.quaternion_matrix(rot))
            mat[0:3,3] =  np.matrix(trans).T
            new_pose = mat*self.offset
            
            trans,rot = mat_to_pos_and_quat(new_pose)
            return trans,rot
        except:
            return False, False

        """
        quat = np.array(rot)
        quat /= np.linalg.norm(quat)
        return trans, list(quat)
        """
if __name__=="__main__":
    rospy.init_node("testing_tf")
    tf_transformer = TfTransformer()
    raw_input("print output?")
    print tf_transformer.get_transform( 'base_link', 'ar_marker_1')

