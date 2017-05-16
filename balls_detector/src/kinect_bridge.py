#!/usr/bin/env python

'''
Borrowed from the libfreenect python wrappers

We get the raw image from the kiunect rgb camera and broadcast its
pixels array on a topic called /kinect/rgb for onward processing
by other nodes

Authors:
    1.  Olalekan Ogunmolu
    2.  Kaveh Fathian
    3.  Sleiman Safaoui

May 11, 2016
'''

from __future__ import print_function

import freenect

#import sys files
import cv2

#ros imports
import rospy
import roslib
roslib.load_manifest('balls_detector')
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:

  def __init__(self):

    self.bridge = CvBridge()
    self.keep_running = True
    self.kinect_pub = rospy.Publisher("/kinect/image/rgb", Image, queue_size=10)

  def get_rgb(self, dev, data, timestamp):
        # global keep_running
        self.kinect_image = data[:, :, ::-1]  # RGB -> BGR

        try:
          self.kinect_pub.publish(self.bridge.cv2_to_imgmsg(self.kinect_image, "bgr8"))
        except CvBridgeError as e:
          print(e)

  def body(self, *args):
        if not self.keep_running:
            raise freenect.Kill

def main():
    rospy.init_node("kinect_rgb_pub")
    ic = ImagePublisher()
    freenect.runloop(video=ic.get_rgb,
                     body=ic.body)

if __name__ == '__main__':
    main()
