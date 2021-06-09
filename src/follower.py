#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        self.twist  = Twist()

        cv2.namedWindow("rgb", 1)
        cv2.namedWindow("hsv", 1)
        cv2.namedWindow("masked", 1)
         
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('camera/image', Image)
            rgb, hsv, masked  = self.process_img(data)
            cv2.imshow("rgb", rgb)
            cv2.imshow("hsv", hsv)
            cv2.imshow("masked", masked)
            cv2.waitKey(3)
            


    def process_img(self, data):
        #color filtering
        image = self.bridge.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        lower_yellow = numpy.array([60, 0, 0])
        upper_yellow = numpy.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)


        #computing moments
        h,w,d = image.shape
        search_top = int(3*h/4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0 :
            cx = int (M['m10']/ M['m00'])
            cy = int (M['m01']/ M['m00'])
            cv2.circle(masked, (cx,cy), 20, (0,255,255), -1)


            #follow the dot/ publish to cmd_vel
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z  = -float(err) / 100
            self.vel_pub.publish(self.twist)


        return rgb, hsv, masked


rospy.init_node('follower')
follower = Follower()
rospy.spin()
