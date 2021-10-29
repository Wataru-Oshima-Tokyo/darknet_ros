#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import threading

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_brdige import CvBridge, CvBridgeError
import cv2
import numpy as np


class ContourDetector():
    def __init__(self):
        self.cv_bridge = CvBridge()
        frameWidth = 640
        frameHeight = 480

        cap = cv2.VideoCapture('videos/rectangle.mp4')
        #cap = cv2.VideoCapture(0)

        # Properties
        cap.set(3, frameWidth)
        cap.set(4, frameHeight)
        cv2.namedWindow('Parameters')
        cv2.resizeWindow('Parameters', 1600, 400)
        cv2.createTrackbar('Threshold1', 'Parameters', 0, 255, self.empty)
        cv2.createTrackbar('Threshold2', 'Parameters', 0, 255, self.empty)
        cv2.createTrackbar('Area', 'Parameters', 5000, 30000, self.empty)
        # Subscribe
        self.sub_camera_rgb     =  rospy.Subscriber('/camera/color/image_raw', Image, self.CamRgbImageCallback)
        return 

    def empty(self, a):
        pass

    def stackImages(self, scale, imgArray):
        rows = len(imgArray)
        cols = len(imgArray[0])
        rowsAvailable = isinstance(imgArray[0], list)
        width = imgArray[0][0].shape[1]
        height = imgArray[0][0].shape[0]
        if rowsAvailable:
            for x in range(0, rows):
                for y in range(0, cols):
                    if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                    else:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                    None, scale, scale)
                    if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
            imageBlank = np.zeros((height, width, 3), np.uint8)
            hor = [imageBlank] * rows
            hor_con = [imageBlank] * rows
            for x in range(0, rows):
                hor[x] = np.hstack(imgArray[x])
            ver = np.vstack(hor)
        else:
            for x in range(0, rows):
                if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                    imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
                else:
                    imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
                if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
            hor = np.hstack(imgArray)
            ver = hor
        return ver

    def getContours(self, img, imgContour):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #cv2.findContours(入力画像、contour retrieval mode, 輪郭研修津方法）
        for cnt in contours:
            area = cv2.contourArea(cnt)
            areaMin = cv2.getTrackbarPos("Area", "Parameters")
            if area > areaMin:
                #輪郭描画
                cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                print(len(approx))
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)

                cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                            (0, 255, 0), 2)
                cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                            (0, 255, 0), 2)
    def CamRgbImageCallback(self,rgb_image_data):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
            imgContour = img.copy()

            # Gaussian Blur
            imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
            # Gray
            imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
            # Canny
            threshold1 = cv2.getTrackbarPos('Threshold1', 'Parameters')
            threshold2 = cv2.getTrackbarPos('Threshold2', 'Parameters')
            imgCanny = cv2.Canny(imgGray, threshold1=threshold1, threshold2=threshold1)
            # Dialation
            kernel = np.ones((5, 5))
            imgDil = cv2.dilate(imgCanny, kernel=kernel, iterations=1)
            #Contours
            self.getContours(imgDil, imgContour)


            imgStack = self.stackImages(0.8, ([img, imgBlur, imgGray],[imgCanny, imgDil, imgContour]))
            cv2.imshow('Result', imgStack)
        except CvBridgeError, e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        rospy.init_node('person_detector', anonymous=True)
        cd = ContourDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
