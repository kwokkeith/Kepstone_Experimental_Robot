#!/usr/bin/env python
from onnx_infer import load_model_data, run_inference, image_point_to_ground_3d
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from decimal import Decimal
from geometry_msgs.msg import PointStamped
import os
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
import time
import pickle
import struct
import socket
import threading


# This class is used to detect object (ball) and publish to camera_frame/detected_object_coordinates
# Sub:
#   /camera/color/image_raw
# Pub:
#   /camera_frame_detected_object_coordinates
# Node Param:  
#   camera_frame


# Pipeline to configure the camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline_profile = pipeline.start(config)

# Get the depth stream intrinsics
#depth_stream = pipeline_profile.get_stream(rs.stream.depth) # Fetch the depth stream 
#intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()


# Stop the pipeline
pipeline.stop()

# Set camera_matrix
camera_matrix = np.array([[390.8465270996094, 0, 323.46539306640625],
                        [0, 390.8465270996094, 242.03125],
                        [0, 0, 1]])
dist_coeffs = np.zeros(5)  # assume no distortion
camera_height = 1.2  # 1.2 meters above the ground
tilt_angle_deg = 30  # 30-degree tilt

rospy.init_node('ball_detector', anonymous=True)
#rate = rospy.Rate(1000000)

# Get global paramaters for topics/services
#detected_object_coordinates_topic_pub_ = rospy.get_param('/litter_detection/topics/detected_object_coordinates_camera')
#camera_frame = rospy.get_param('~camera_frame', 'dpcamera_link')

# Publish detected object coordinates
#coord_publisher = rospy.Publisher(detected_object_coordinates_topic_pub_, PointStamped, queue_size=10)
coord_publisher = rospy.Publisher("/camera_frame/detected_object_coordinates", PointStamped, queue_size=10)

HOST = "192.168.0.151" # Jetson static ip
PORT = 5555

client_socket = None

def run_example(frame):   
    # Load image
    input_image = frame

    # Run inference
    boxes, scores, class_names = run_inference(
    input_img=input_image, score_thr=0.05, input_shape="480,640")

    detection_results = []

    for i in range(len(boxes)):
        target_point = ((boxes[i][0]+boxes[i][2])/2, (boxes[i][1]+boxes[i][3])/2)  # Example target point in the image
        # the distance is ground_position[2]
        ground_position = image_point_to_ground_3d(target_point, camera_matrix, dist_coeffs, camera_height, tilt_angle_deg)

        # Create a PointStamped message
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()     # Add a timestamp
        #point_msg.header.frame_id = camera_frame # Indicate the frame of reference
        point_msg.header.frame_id = "base_link" # TODO: Temporary for testing

        # Point coordinates are in robot coordinate system (point) as opposed to the 
        # optical coordinate system of camera (Xtarget, Ytarget, Ztarget)
        point_msg.point.x = ground_position[2]
        point_msg.point.y = -ground_position[0]
        point_msg.point.z = ground_position[1]

        coord_publisher.publish(point_msg)        

        # Store detection info
        detection_results.append({
            "class": class_names[i],
            "score": scores[i],
            "bbox": boxes[i],
            "ground_position": (ground_position[2], -ground_position[0], ground_position[1])
        })
        
    return detection_results
         
def socket_server():
    """ Function to handle socket connections in a non-blocking way """
    global client_socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen(5)
    sock.setblocking(False)  # Make socket non-blocking

    print(f"[*] Server started at {HOST}:{PORT}, waiting for a connection...")

    while not rospy.is_shutdown():
        try:
            new_client, client_address = sock.accept()  # Non-blocking accept
            print(f"[*] Accepted connection from {client_address}")
            client_socket = new_client  # Store active client socket
            client_socket.setblocking(True)  # Client socket should be blocking for sending
        except BlockingIOError:
            time.sleep(1)  # No client connected yet, continue detection
        except Exception as e:
            print(f"[ERROR] {e}")
            time.sleep(5)

    sock.close()

def main():
    """ Main function to start OpenCV processing and socket server in parallel """
    global client_socket

    current_dir = os.path.dirname(os.path.realpath(__file__))
    model_path = os.path.join(current_dir, "../models/encrypt-model-v1-fp16.7z")
    load_model_data(model_path)

    # Start socket server in a separate thread
    socket_thread = threading.Thread(target=socket_server, daemon=True)
    socket_thread.start()

    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("Could not open video device")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture video frame")
            break

        detections = run_example(frame)  # Always run inference

        # --- Compress and prepare data ---
        _, encoded_frame = cv2.imencode(".jpg", frame)
        frame_data = encoded_frame.tobytes()

        data_packet = {
            "image": frame_data,
            "detections": detections
        }
        serialized_data = pickle.dumps(data_packet)
        message_size = struct.pack("!I", len(serialized_data))  # Fixed 4-byte size header

        # --- Send data if client is connected ---
        if client_socket:
            try:
                client_socket.sendall(message_size + serialized_data)
            except (BrokenPipeError, ConnectionResetError):
                print("[!] Client disconnected.")
                client_socket = None  # Reset client socket

        #cv2.imshow("Model", frame)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

