#!/usr/bin/env python

'''
We subscribe to the kinect rgb camera topic and perform some detection

Authors:
    1.  Olalekan Ogunmolu
    2.  Kaveh Fathian
    3.  Sleiman Safaoui

May 11, 2016
'''

from __future__ import print_function

import cv2
import argparse

#ros imports
import rospy
import roslib
import numpy as np
roslib.load_manifest('balls_detector')
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

parser = argparse.ArgumentParser(description='Process environmental variables')
parser.add_argument("--disp", default=True, action="store_true")
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--width", type=int, default=640)
args = parser.parse_args()

class ImageConverter:

  def __init__(self, args):
    self.bridge = CvBridge()
    self.kinect_image = np.zeros((args.height, args.width))  #default initialization
    self.image_sub = rospy.Subscriber("/kinect/image/rgb",Image,self.callback, queue_size=1)
    self.depth_sub = rospy.Subscriber("/kinect/image/depth",Image,self.depth_call, queue_size=1)

  def callback(self,data):
    try:
      self.kinect_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def depth_call(self, data):
    # TODO
    try:
      self.kinect_depth = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def get_image(self):
    return self.kinect_image

  def shutdown_hook(self):
    print("shutting down!")

  def resize_frame(self, frame):
      '''
      override this in derived classes
      '''
  def display_image(self, image):
      '''
      override
      '''

  def erode_dilate(self, frame):
      '''
      override this in derived classes
      '''

  def detect_blobs(self):
      '''
      override this in derived classes
      '''

  def prepro(self, frame):
      '''
      override this in derived classes
      '''

class SpheroDetector(ImageConverter):
    """
    We perform morphological properties,
    detect blobs and blah, blah, blah
    """
    def __init__(self, args, numRob):
        ImageConverter.__init__(self, args)

        #initialize variables
        self.numRob             = numRob
        self.locs               = np.zeros((2, numRob))#, dtype=np.int))
        self.bboxes             = np.zeros((4, numRob))#, dtype=np.int));
        self.thresh             = 127;
        self.numBlobsChecked    = 0;
        self.newDetectedTotal   = 0;
        self.centersTemp        = np.zeros((2, 1))
        self.bboxesTemp         = np.zeros((4, 1))

    #resizing the image
    def resize_frame(self, frame):
        return cv2.resize(frame, (0, 0), fx=1.0, \
                         fy=1.0, interpolation=cv2.INTER_NEAREST)

    #erode and dilate the image to remove noise
    def erode_dilate(self, frame):
        anchor = (-1, -1)  #means position of anchor is at the center
        ksize  = (4,4) # Size of the structuring element.
        # structuring element used for erosion
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, ksize, anchor)
        cv2.erode(frame, kernel, frame, anchor, iterations=2)
        cv2.dilate(frame, kernel, frame, anchor, 2)

    def display_image(self, image):
        image = self.resize_frame(image)

        k = 0xFF & cv2.waitKey(1)
        cv2.imshow('blobbed images', image)

        if k == 27:
            cv2.destroyAllWindows()
            rospy.on_shutdown(self.shutdown_hook)

    def detect_blobs(self):
        '''
        This is a useful tutorial:
        https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        '''

        #preprocess image
        def prepro(frame):
            #resize the image
            frame = self.resize_frame(frame)

            frame *= 1/255
            frame = (frame/255).astype('uint8')
            print(frame.shape)
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, frame);

            self.display_image(frame)

            #binarize image
            _, frame = cv2.threshold(frame, self.thresh, 255, cv2.THRESH_BINARY)


            #erode and dilate image
            self.erode_dilate(frame)

            return frame

        self.image = self.get_image()

        #pre-process images first
        self.image = prepro(self.image)

        #setup detector Params
        params = cv2.SimpleBlobDetector_Params()

        #change thresholds
        params.minThreshold = 20;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.85

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.85

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else :
            detector = cv2.SimpleBlobDetector_create(params)

        #Detect blobs
        keypoints = detector.detect(self.image)

        '''
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the
        size of the circle corresponds to the size of blob
        '''
        # im_with_keypoints = cv2.drawKeypoints(image, keypoints, \
        #                                       np.array([]), (0,0,255), \
        #                                       cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


def main():
    rospy.init_node("kinect_subscriber_node")
    rate = rospy.Rate(30)
    global args
    sd = SpheroDetector(args, 2)
    try:
        while not rospy.is_shutdown():

            sd.detect_blobs()
            rate.sleep()
        # rospy.spin()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    main()
