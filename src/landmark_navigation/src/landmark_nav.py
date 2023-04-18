#!/usr/bin/env python
from __future__ import print_function
from select import select

import sys
import threading
import yaml

import rospy

from move_base_msgs.msg import *
from geometry_msgs.msg import PointStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


class LandmarkNav(threading.Thread):
    '''.'''
    def __init__(self, rate):
        super(LandmarkNav, self).__init__()
        rospy.on_shutdown(self.cancel)

        # Publishes the clicked_point for send_mark to take over.
        self.publisher = rospy.Publisher(
            'clicked_point', PointStamped, queue_size=1
        )

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()
        
    def wait_for_subscribers(self):
        '''The node waits till it has >= 1 subscriber before running.'''
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print(
                    "Waiting for subscriber to connect to {}.".format(
                        self.publisher.name
                    )
                )
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected.")

    def publish_target_coordinates(self, x, y):
        '''Publishes the target coordinates.'''
        msg = PointStamped()
        msg.point.x = x
        msg.point.y = y
        self.publisher.publish(msg)

    # def stop(self):
    #     '''.'''
    #     self.done = True
    #     self.join()

    def cancel(self):
        '''.'''
        self.publisher.unregister()

def getKey(settings, timeout):
    '''.'''
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    '''.'''
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    '''.'''
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    # Get the landmarks_file argument.
    if len(sys.argv) < 2:
        rospy.logwarn(
            "Incorrect number of arguments. Using default landmarks file "
            "landmarks/landmarks.yaml"
        )
    landmarks_file = "landmarks/landmarks.yaml"
    # else:
    #     exit()
    # print("\n\n")
    # landmarks_file = rospy.get_param("/landmark_nav/landmarks_file")
    # print("LANDMARKS FILE: ".format(landmarks_file))

    # Load the landmarks file.
    with open(landmarks_file, 'r') as file:
        landmarks_dict = yaml.safe_load(file)
    landmarks = list(landmarks_dict.keys())

    # Create the landmarks list portion of the CLI message.
    landmarks_msg = ""
    for landmark_idx, landmark in enumerate(landmarks):
        if landmark_idx < len(landmarks)-1:
            landmarks_msg += "\t{}\n".format(landmark)
        else:
            landmarks_msg += "\t{}".format(landmark)

    # The node's CLI message.
    msg = """
Landmark Navigation
---------------------------
The map has the following landmarks:
{}

Type the desired landmark # to start navigation to it.

CTRL-C to quit.
    """.format(landmarks_msg)

    settings = saveTerminalSettings()
    rospy.init_node('landmark_nav')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = LandmarkNav(repeat)

    try:
        pub_thread.wait_for_subscribers()
        print(msg)
        while(1):
            key = getKey(settings, key_timeout)
            # key = int(raw_input("Enter "))
            try:
                key = int(key)
            except:
                pass
            print("{}".format(key), end='\r')

            if key in landmarks:
                print("Navigating to landmark {} as requested.".format(key))

                # Publish the desired coordinates to clicked_point.
                pub_thread.publish_target_coordinates(
                    landmarks_dict[key]["x"],
                    landmarks_dict[key]["y"]
                )
                # break
            elif (key == '\x03'):
                break
            else:
                print("Not a valid landmark number.", end='\r')
    except Exception as e:
        print(e)
    finally:
        # pub_thread.stop()
        restoreTerminalSettings(settings)