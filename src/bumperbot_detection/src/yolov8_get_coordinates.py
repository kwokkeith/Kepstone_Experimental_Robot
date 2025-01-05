#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from decimal import Decimal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters  # To synchronize color and depth frames
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import os


# This class is used to detect object (ball) and publish to camera_frame/detected_object_coordinates
# Sub:
#   /camera/color/image_raw
# Pub:
#   /camera_frame_detected_object_coordinates
# Node Param:  
#   camera_frame

class BallDetector:
    def __init__(self):
        ## Get global paramaters for topics/services
        detected_object_coordinates_topic_pub_ = rospy.get_param('/litter_detection/topics/detected_object_coordinates_camera')
        camera_color_image_raw_client = rospy.get_param('/camera/topics/color_image_raw')
        camera_depth_image_raw_client = rospy.get_param('/camera/topics/depth_image_raw')

        current_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(current_dir, "../models/yolov8.pt")
        self.model = YOLO(model_path)
        
        self.bridge_object = CvBridge()
        self.theta = 0.2618  # 30 degrees in radians

        # Get the camera frame from the ROS parameter server (default to 'dpcamera_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'dpcamera_link')

        self.coord_publisher = rospy.Publisher(detected_object_coordinates_topic_pub_, PointStamped, queue_size=10)

        # Subscribe to color and depth topics
        self.color_sub = message_filters.Subscriber(camera_color_image_raw_client, Image)
        self.depth_sub = message_filters.Subscriber(camera_depth_image_raw_client, Image)

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
            # dist = depth_value * 1000  # Convert depth from meters to mm
            dist = depth_value           # Depth in mm

            # Calculate X, Y, Z real-world coordinates
            Xtemp = dist * (x - self.intrinsics['cx']) / self.intrinsics['fx']
            Ytemp = dist * (y - self.intrinsics['cy']) / self.intrinsics['fy']
            Ztemp = dist

            Xtarget = Xtemp  # Adjust for RGB camera module offset
            Ytarget = -(Ztemp * math.sin(self.theta) + Ytemp * math.cos(self.theta))
            Ztarget = Ztemp * math.cos(self.theta) - Ytemp * math.sin(self.theta)

            # Create a PointStamped message
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()     # Add a timestamp
            point_msg.header.frame_id = self.camera_frame # Indicate the frame of reference
            
            # Point coordinates are in robot coordinate system (point) as opposed to the 
            # optical coordinate system of camera (Xtarget, Ytarget, Ztarget)
            point_msg.point.y = -Xtarget
            point_msg.point.z = Ytarget
            point_msg.point.x = Ztarget

            self.coord_publisher.publish(point_msg)

            # Display coordinates on image with up to 3 decimal places
            target_coordinates = f"({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f})"

            # Add the text with the coordinates to the image
            cv2.putText(image, target_coordinates, (x - 160, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


        # Calculate and display FPS
        self.new_frame_time = rospy.Time.now().to_sec()

        # Check for division by 0
        if (self.new_frame_time - self.prev_frame_time) > 0:
            fps = 1 / (self.new_frame_time - self.prev_frame_time)
        else:
            fps = 0  # Default value to avoid division by zero
        fps_text = str(int(fps))
        cv2.putText(image, fps_text, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 3, cv2.LINE_AA)
        self.prev_frame_time = self.new_frame_time

        # Show the image with detections
        # cv2.imshow('result', image)
        # return Xtarget, Ytarget, Ztarget
        self.last_image = image

    def detect_image(self, color_image):

        center_coordinates_array = []
        results = self.model(color_image)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                confidence_text = f"{confidence:.2f}"

                if confidence < 0.2:  
                    continue

                # Get the center point of the bounding box
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                center_coordinates_array.append([x_center, y_center])

                cv2.rectangle(color_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
                cv2.putText(color_image, (f"CONFIDENCE:{confidence_text}"), (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                

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
