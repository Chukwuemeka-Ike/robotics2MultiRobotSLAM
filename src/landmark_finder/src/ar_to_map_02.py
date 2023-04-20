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

def distance(pos1, pos2):
    #takes two 3x1 array, return their distance
    tmp = pow(pos1[0]-pos2[0],2)
    tmp += pow(pos1[1]-pos2[1],2)
    tmp += pow(pos1[2]-pos2[2],2)
    return math.sqrt(tmp)


class Process:
    def __init__(self):
        self.marker = None
        self.id = None
        self.local = None
        self.landmarks = []
        self.landmarks.append((1,np.array([0, 0, 0])))

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

    def save_landmarks(self):
        print('shutting down, saving landmarks')
        with open('landmarks.txt', 'w') as f:
            for landmark in self.landmarks:
                line1 = 'id: ' + str(landmark[0])
                f.write(line1)
                line2 = 'pos: ' + str(landmark[1])
                f.write(line2)
                # print(line1)
                # print(line2)
        print('landmarks size:',len(self.landmarks))
        f.close()
        


if __name__ == '__main__':
    rospy.init_node('find_ar_mark')
    listener = tf.TransformListener()

    process = Process()

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers , process.marker_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, process.amcl_callback)
    rospy.on_shutdown(process.save_landmarks)
    
    
    while not rospy.is_shutdown():
        #print('id:',process.id)
        #print('marker:')
        #print(process.marker)
        #print('amcl:')
        #print(process.local)
        rospy.sleep(1)
        if process.local != None and process.marker != None:
            # try:
            #     (trans, rot) = listener.lookupTransform('/ar_marker_12', '/usb_cam', rospy.Time(0))
            #     print(trans)
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     pass

            localQuat = np.array([process.local.orientation.w, process.local.orientation.x,
                                  process.local.orientation.y, process.local.orientation.z])
            localRot = q2R(localQuat)
            localPos = np.array([process.local.position.x, process.local.position.y,
                                 process.local.position.z])
            markerPos = np.array([process.marker.position.z, -process.marker.position.x,
                                  -process.marker.position.y])

            worldPos = localPos + localRot.dot(markerPos)
            
            print(process.id)
            print(localRot)
            print(localPos)
            print(markerPos)
            print(worldPos)
            
            #landmark save in the form: (id,[x,y,z])
            
            for landmark in process.landmarks:
                if distance(landmark[1], worldPos) > 0.3:
                    process.landmarks.append((process.id, worldPos))
            if len(process.landmarks) == 0:
                process.landmarks.append((process.id, worldPos))

        else:
            print('marker:')
            print(process.marker)
            print('amcl:')
            print(process.local)
    #rospy.spin()