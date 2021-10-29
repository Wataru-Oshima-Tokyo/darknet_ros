#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading

# This import is for ROS integration
import rospy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2

class PersonDetector():
    def __init__(self):

        # cv_bridge handles
        self.cv_bridge = CvBridge()

        self.person_bbox = BoundingBox()

        # ROS PARAM
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.40)

        # Subscribe
        self.sub_camera_rgb     =  rospy.Subscriber('/camera/color/image_raw', Image, self.CamRgbImageCallback)
        # self.sub_camera_depth   =  rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.CamDepthImageCallback)
        self.sub_darknet_bbox   =  rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBboxCallback)
        self.image_pub          =  rospy.Publisher('/camera/yolo/image_raw', Image, queue_size=1)
        self.distance           =  rospy.Publisher('/camera/yolo/distance', String, queue_size=1)
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
        
    def CamRgbImageCallback(self, rgb_image_data):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
            imgContour = rgb_image.copy()
        except CvBridgeError, e:
            rospy.logerr(e)

        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # 人がいる場合
        if self.person_bbox.probability > 0.0 :

           # 一旦、BoundingBoxの中心位置の深度を取得 (今後改善予定）
            cv2.rectangle(rgb_image, (self.person_bbox.xmin, self.person_bbox.ymin), (self.person_bbox.xmax, self.person_bbox.ymax),(0,0,255), 2)
            text=""
            text_top = (self.person_bbox.xmin, self.person_bbox.ymin - 10)
            text_bot = (self.person_bbox.xmin + 80, self.person_bbox.ymin + 5)
            text_pos = (self.person_bbox.xmin + 5, self.person_bbox.ymin)
            cv2.rectangle(rgb_image, text_top, text_bot, (0,0,0),-1)
            cv2.putText(rgb_image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(rgb_image)) 
                        

            # Gaussian Blur
            imgBlur = cv2.GaussianBlur(rgb_image, (7, 7), 1)
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


            imgStack = self.stackImages(0.8, ([rgb_image, imgBlur, imgGray],[imgCanny, imgDil, imgContour]))
            cv2.imshow('Result', imgStack)
            cv2.namedWindow("rgb_image")
            cv2.imshow("rgb_image", rgb_image)
            cv2.waitKey(1)
#         cv2.waitKey(10)
#         cv2.normalize(self.m_depth_image, self.m_depth_image, 0, 32768, cv2.NORM_MINMAX)
#         cv2.namedWindow("depth_image")
#         cv2.imshow("depth_image", self.m_depth_image)
#         cv2.waitKey(10)
          
        return

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

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        person_bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'person' and bboxs[i].probability >= self.m_pub_threshold:
                    person_bbox = bboxs[i]        
        self.person_bbox = person_bbox



if __name__ == '__main__':
    try:
        rospy.init_node('person_detector', anonymous=True)
        idc = PersonDetector()
        rospy.loginfo('idc Initialized')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
