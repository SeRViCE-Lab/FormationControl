#!/usr/bin/env python

# from __future__ import print_function

"""
    OpenCV sphero_detector
    Implements Kaveh's Sphero detector algorithm
    April 27, 2017
"""
__author__ = 'Olalekan Ogunmolu <Olalekan.Ogunmolu@utdallas.edu>'
__version__ = '0.0'
__license__ = 'MIT'

import cv2.cv as cv
import cv2
import time
import roslib
import rospy

roslib.load_manifest('sphero_run')

class Sphero_Detector(object):
    def __init__(self):

        self.windowName = "sphero_detection"
        # self.getImage()
        print("spinning")

    def getImage(self):
        cv.NamedWindow(self.windowName, 1)
        capture = cv.CaptureFromCAM(0)

        img = cv.QueryFrame(capture)
        cv.ShowImage(self.windowName, img)
        if cv.WaitKey(10) == 27:
            cv.destroyAllWindows()

def main():
    rospy.init_node("detector_node", anonymous=True)
    sd = Sphero_Detector()
    try:
        while not rospy.is_shutdown():
            sd.getImage()
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    main()
