#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading

from numpy.lib.function_base import median

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
        self.sub_camera_depth   =  rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.CamDepthImageCallback)
        self.sub_darknet_bbox   =  rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBboxCallback)
        self.image_pub          =  rospy.Publisher('/camera/yolo/image_raw', Image, queue_size=1)
        self.distance           =  rospy.Publisher('/camera/yolo/distance', String, queue_size=1)
        self.direction          =  rospy.Publisher('/camera/yolo/direction', String, queue_size=1)
        return

    def CamRgbImageCallback(self, rgb_image_data):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # 人がいる場合
        if self.person_bbox.probability > .3 :
            center_screen_x = 480/2
           # 一旦、BoundingBoxの中心位置の深度を取得 (今後改善予定）
            center_x, center_y= (int)(self.person_bbox.xmax+self.person_bbox.xmin)/2, (int)(self.person_bbox.ymax+self.person_bbox.ymin)/2
            min_x, min_y = center_x-20, center_y-20
            max_x, max_y = center_x+20, center_y+20
            diff_x = center_screen_x - center_x
	    m_person_depth = self.m_depth_image[center_x][center_y]
            distance =[]
	    distance.append(m_person_depth)
	    try:
                # boxArray = np.array[(int)(self.person_bbox.xmin):(int)(self.person_bbox.xmax), (int)(self.person_bbox.ymin):(int)(self.person_bbox.ymax)]
                for i in range(min_x, max_x):
                    for j in range(min_y, max_y):
                        #if self.m_depth_image[int(i)][int(j)] !=0 and self.m_depth_image[int(i)][int(j)] <2000:
                        distance.append(self.m_depth_image[int(i)][int(j)])
                m_person_depth = median(distance)
            except Exception as e:
                print(e)
                m_person_depth = median(distance)
                pass
            
            cv2.rectangle(rgb_image, (self.person_bbox.xmin, self.person_bbox.ymin), (self.person_bbox.xmax, self.person_bbox.ymax),(0,0,255), 2)
            rospy.loginfo('Class : person, Score: %.2f, Dist: %dmm ' %(self.person_bbox.probability, m_person_depth))
            text = "person " +('%dmm' % m_person_depth)
            text_top = (self.person_bbox.xmin, self.person_bbox.ymin - 10)
            text_bot = (self.person_bbox.xmin + 80, self.person_bbox.ymin + 5)
            text_pos = (self.person_bbox.xmin + 5, self.person_bbox.ymin)
            cv2.rectangle(rgb_image, text_top, text_bot, (0,0,0),-1)
            cv2.putText(rgb_image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
            self.distance.publish(str(m_person_depth))
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(rgb_image))
            self.direction.publish(str(diff_x))
#         cv2.namedWindow("rgb_image")
#         cv2.imshow("rgb_image", rgb_image)
#         cv2.waitKey(10)
#         cv2.normalize(self.m_depth_image, self.m_depth_image, 0, 32768, cv2.NORM_MINMAX)
#         cv2.namedWindow("depth_image")
#         cv2.imshow("depth_image", self.m_depth_image)
#         cv2.waitKey(10)
          
        return


    def CamDepthImageCallback(self, depth_image_data):
        try:
            self.m_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.m_camdepth_height, self.m_camdepth_width = self.m_depth_image.shape[:2]
        return

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
