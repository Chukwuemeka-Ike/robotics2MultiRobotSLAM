#!/usr/bin/env python
# reference from general_robotics_toolbox.py
# Copyright (c) 2018, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 


import rospy
import math
import numpy as np
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped

# map1 name: hectorLaserCarMonoSlow
# map2 name: team4_map

def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector
             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]
    :type    k: numpy.array
    :param   k: 3 x 1 vector
    :rtype:  numpy.array
    :return: the 3 x 3 cross product matrix    
    """
    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]    
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix according to the
    Euler-Rodrigues formula.
    :type    q: numpy.array
    :param   q: 4 x 1 vector representation of a quaternion q = [q0;qv]
    :rtype:  numpy.array
    :return: the 3x3 rotation matrix    
    """
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2;

class Process:
    def __init__(self):
        self.marker = None
        self.id = None
        self.local = None
        

    def marker_callback(self, msg):
        # "Store" message received.
        if len(msg.markers):
            self.marker = msg.markers[0].pose.pose
            self.id = msg.markers[0].id
        else:
            self.marker = None;
            self.id = None;
            

    def amcl_callback(self, msg):
        # "Store" the message received.
        self.local = msg.pose.pose
        


if __name__ == '__main__':
    rospy.init_node('find_ar_mark')
    listener = tf.TransformListener()

    process = Process()

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers , process.marker_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, process.amcl_callback)
    
    
    while not rospy.is_shutdown():
        #print('id:',process.id)
        #print('marker:')
        #print(process.marker)
        #print('amcl:')
        #print(process.local)
        rospy.sleep(1)
        if process.local != None and process.marker != None:
            try:
                (trans, rot) = listener.lookupTransform('/usb_cam', '/base_link', rospy.Time(0))
                print(trans)
                rot = q2R(rot)
                transRobotToCamera = np.concatenate((rot, trans), axis=1)
                transRobotToCamera = np.concatenate((
                    transRobotToCamera, np.concatenate((np.zeros((1,3)), 1), axis=1)
                ))

                localQuat = np.array([process.local.orientation.w, process.local.orientation.x,
                                    process.local.orientation.y, process.local.orientation.z])
                localRot = q2R(localQuat)
                localPos = np.array([process.local.position.x, process.local.position.y,
                                    process.local.position.z])
                markerQuat = np.array([process.marker.orientation.w, process.marker.orientation.x,
                                    process.marker.orientation.y, process.marker.orientation.z])
                markerRot = q2R(markerQuat)
                markerPos = np.array([process.marker.position.z, -process.marker.position.x,
                                    -process.marker.position.y])

                worldPos = localPos + localRot.dot(markerPos)

                transWorldToRobot = np.concatenate((localRot, localPos), axis=1)
                transWorldToRobot = np.concatenate((
                    transWorldToRobot, np.concatenate((np.zeros((1,3)), 1), axis=1)
                ))
                transCameraToTag = np.concatenate((markerRot, markerPos), axis=1)
                transCameraToTag = np.concatenate((
                    transCameraToTag, np.concatenate((np.zeros((1,3)), 1), axis=1)
                ))
                
                a = transWorldToRobot*transRobotToCamera*transCameraToTag
                print(a[-1, :])

                print(process.id)
                print(localRot)
                print(localPos)
                print(markerPos)
                print(worldPos)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass


        else:
            print('marker:')
            print(process.marker)
            print('amcl:')
            print(process.local)
    #rospy.spin()

    
    
    
#[ 0.76537532 -0.75892303  1.27043714]