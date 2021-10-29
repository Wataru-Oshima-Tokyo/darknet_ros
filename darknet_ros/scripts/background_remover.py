import os
import threading

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class PersonContourDetector():
    def __init__(self):

        # cv_bridge handles
        self.cv_bridge = CvBridge()

        # ROS PARAM
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.40)

        # Subscribe
        self.sub_camera_rgb     =  rospy.Subscriber('/camera/color/image_raw', Image, self.remove_bg)
 

    def remove_bg(self, rgb_image_data):
        BLUR = 21,
        CANNY_THRESH_1 = 10,
        CANNY_THRESH_2 = 200,
        MASK_DILATE_ITER = 10,
        MASK_ERODE_ITER = 10,
        MASK_COLOR = (0.0,0.0,1.0),
        try:
            img = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        #color ->gray
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        #find the edge
        edges = cv2.Canny(gray, CANNY_THRESH_1, CANNY_THRESH_2)
        edges = cv2.dilate(edges, None)
        edges = cv2.erode(edges, None)
        

        #find the contour
        contour_info = []
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for c in contours:
            contour_info.append((
                c,
                cv2.isContourConvex(c),
                cv2.contourArea(c),
            ))
        contour_info = sorted(contour_info, key=lambda c: c[2], reverse=True)
        max_contour = contour_info[0]

        # create empty mask, draw filled polygon on it corresponding to the largest contour 
        mask = np.zeros(edges.shape)
        cv2.fillConvexPoly(mask, max_contour[0], (255))

        #smoothing the mask and make it to 3 channels
        mask = cv2.dilate(mask, None, iterations=MASK_DILATE_ITER)
        mask = cv2.erode(mask, None, iterations=MASK_ERODE_ITER)
        mask = cv2.GaussianBlur(mask, (BLUR, BLUR), 0)
        mask_stack = np.dstack([mask]*3)    # Create 3-channel alpha mask

        
        mask_stack  = mask_stack.astype('float32') / 255.0          # Use float matrices, 
        img         = img.astype('float32') / 255.0                 #  for easy blending

        masked = (mask_stack * img) + ((1-mask_stack) * MASK_COLOR) # Blend
        masked = (masked * 255).astype('uint8')                     # Convert back to 8-bit 

        c_blue, c_green, c_red = cv2.split(img)

        img_a = cv2.merge((c_red, c_green, c_blue, mask.astype('float32') / 255.0))


        cv2.imshow('Resutl', img_a)
        return img_a
if __name__ == '__main__':
    try:
        rospy.init_node('person_contour_detector', anonymous=True)
        idd = PersonContourDetector()
        rospy.loginfo('idd Initialized')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

