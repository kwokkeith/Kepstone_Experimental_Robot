#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from decimal import Decimal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters  # To synchronize color and depth frames
from std_msgs.msg import Float32MultiArray

class BallDetector:
    def __init__(self):
        self.bridge_object = CvBridge()
        self.theta = 0.2618  # 30 degrees in radians

        self.coord_publisher = rospy.Publisher('/object_coordinates', Float32MultiArray, queue_size=10)

        # Subscribe to color and depth topics
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)

        # Synchronize the color and depth messages
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        # Manually added based on gazebo depth camera intrinsics
        self.intrinsics = {
            "fx": 528.43,
            "fy": 528.43,
            "cx": 320,
            "cy": 240
        }  

        # Frame rate variables
        self.new_frame_time = 0
        self.prev_frame_time = 0

        self.last_image = None

    def image_callback(self, color_msg, depth_msg):
        try:
            # Convert ROS image messages to OpenCV images
            color_image = self.bridge_object.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge_object.imgmsg_to_cv2(depth_msg, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        # Perform object detection on color image
        image, center_coordinates_array = self.detect_image(color_image)

        # Calculate real-world coordinates using depth data
        for i in range(len(center_coordinates_array)):
            x = int(center_coordinates_array[i][0])
            y = int(center_coordinates_array[i][1])
            depth_value = depth_image[y, x]  # Get the depth value at (x, y)
            if not np.isfinite(depth_value):  # Skip if depth value is invalid (NaN or inf)
                continue
            dist = depth_value * 1000  # Convert depth from meters to mm

            # Calculate X, Y, Z real-world coordinates
            Xtemp = dist * (x - self.intrinsics['cx']) / self.intrinsics['fx']
            Ytemp = dist * (y - self.intrinsics['cy']) / self.intrinsics['fy']
            Ztemp = dist

            Xtarget = Xtemp - 35  # Adjust for RGB camera module offset
            Ytarget = -(Ztemp * math.sin(self.theta) + Ytemp * math.cos(self.theta))
            Ztarget = Ztemp * math.cos(self.theta) - Ytemp * math.sin(self.theta)

            coord_msg = Float32MultiArray()
            coord_msg.data = [Xtarget, Ytarget, Ztarget]

            self.coord_publisher.publish(coord_msg)

            # Display coordinates on image
            target_coordinates = f"({Decimal(Xtarget).quantize(Decimal('0'))}, " \
                               f"{Decimal(Ytarget).quantize(Decimal('0'))}, " \
                               f"{Decimal(Ztarget).quantize(Decimal('0'))})"
            cv2.putText(image, target_coordinates, (x - 160, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # temp_coordinates = f"({Decimal(Xtemp).quantize(Decimal('0'))}, " \
            #                      f"{Decimal(Ytemp).quantize(Decimal('0'))}, " \
            #                      f"{Decimal(dist).quantize(Decimal('0'))})"
            # cv2.putText(image, temp_coordinates, (x - 100, y + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Calculate and display FPS
        self.new_frame_time = rospy.Time.now().to_sec()
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        fps_text = str(int(fps))
        cv2.putText(image, fps_text, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 3, cv2.LINE_AA)
        self.prev_frame_time = self.new_frame_time

        # Show the image with detections
        # cv2.imshow('result', image)
        # return Xtarget, Ytarget, Ztarget
        self.last_image = image

    def detect_image(self, color_image):
        # convert image to hsv
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # cv.imshow("hsv", hsv_image)

        # values obtained by experimentation to include blue, yellow, orange, green, red
        lower_color = np.array([0, 100, 0])  
        upper_color = np.array([255, 255, 255])  

        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        # cv.imshow("mask", mask)

        # find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("contours: ", contours)

        center_coordinates_array = []

        for contour in contours:
            area = cv2.contourArea(contour)  # cv.contourArea returns a scalar
            threshold = 100  # Define some area threshold to filter small contours
            
            # Ensure area is a scalar and compare it with the threshold
            if area > threshold: 

                # Fit a circle around the contour -> get center coordinates
                (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                center_coordinates_array.append([x_center, y_center])
                min_radius = 10  
                max_radius = 150  
                
                if min_radius < radius < max_radius:
                    # bounding box
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(color_image, [box], 0, (0,255,0), 2)
                

        # cv2.imshow("original", color_image)
        return color_image, center_coordinates_array  # Example: returning a single point in the center of the image

if __name__ == "__main__":
    rospy.init_node('ball_detector', anonymous=True)
    detector = BallDetector()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if detector.last_image is not None:
            # Display the last processed image
            cv2.imshow('Detected Objects', detector.last_image)
            cv2.waitKey(1)  # Add a delay to refresh the display window
        rate.sleep()

    cv2.destroyAllWindows()
