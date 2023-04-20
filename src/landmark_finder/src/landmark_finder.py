#!/usr/bin/env python

import rospy

import tf
import math
import numpy as np
from tf.transformations import quaternion_matrix

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped


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
    rospy.init_node('landmark_finder')
    listener = tf.TransformListener()

    process = Process()

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers , process.marker_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, process.amcl_callback)
    rospy.on_shutdown(process.save_landmarks)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.sleep(1)
        if process.local != None and process.marker != None:
            try:
                # markerQuat = np.array([process.marker.orientation.w, process.marker.orientation.x,
                #                     process.marker.orientation.y, process.marker.orientation.z])
                # rot = quaternion_matrix(markerQuat)
                # rot[0:3, -1] = [process.marker.position.x, process.marker.position.y,
                #                     process.marker.position.z]
                # print(rot)

                # 
                (trans, rot) = listener.lookupTransform('ar_marker_{}'.format(process.id), 'usb_cam', rospy.Time(0))
                # print(trans)
                # print(rot)
                transRobotToTag = quaternion_matrix([rot[3], rot[0], rot[1], rot[2]])
                transRobotToTag[0:3, -1] = trans
                # print(transRobotToTag)

                localQuat = np.array([process.local.orientation.w, process.local.orientation.x,
                                    process.local.orientation.y, process.local.orientation.z])
                transWorldToRobot = quaternion_matrix(localQuat)
                transWorldToRobot[0:3, -1] = [process.local.position.x,
                                              process.local.position.y,
                                              process.local.position.z]
                transWorldToTag = transWorldToRobot*transRobotToTag
                print(transWorldToTag)

                worldPos = transWorldToTag[:3, -1]

                # Save landmark in the form: (id,[x,y,z]).
                for landmark in process.landmarks:
                    if distance(landmark[1], worldPos) > 0.3:
                        process.landmarks.append((process.id, worldPos))
                if len(process.landmarks) == 0:
                    process.landmarks.append((process.id, worldPos))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # print(e)
                # break
                continue
    # rate.sleep()