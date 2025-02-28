#!/usr/bin/env python
from onnx_infer import load_model_data, run_inference, image_point_to_ground_3d
import rospy
import cv2
import numpy as np
import pickle
import socket
import struct
import threading
import time
import os
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

# --- Camera Configuration ---
camera_matrix = np.array([[390.8465270996094, 0, 323.46539306640625],
                        [0, 390.8465270996094, 242.03125],
                        [0, 0, 1]])
dist_coeffs = np.zeros(5)  # assume no distortion
camera_height = 1.2  # 1.2 meters above the ground
tilt_angle_deg = 30  # 30-degree tilt

# --- ROS Parameters ---
rospy.init_node('ball_detector', anonymous=True)
detected_object_coordinates_topic_pub_ = rospy.get_param('/litter_detection/topics/detected_object_coordinates_camera')
camera_frame = rospy.get_param('~camera_frame', 'dpcamera_link')

# --- Publisher ---
coord_publisher = rospy.Publisher(detected_object_coordinates_topic_pub_, PointStamped, queue_size=10)

# --- Socket Parameters ---
HOST = "192.168.0.151"  # Change this to the actual IP of the Jetson
PORT = 5555

def run_example(frame):
    """ Run inference and return detection results """
    boxes, scores, class_names = run_inference(
        input_img=frame, score_thr=0.05, input_shape="480,640"
    )

    detection_results = []

    for i in range(len(boxes)):
        target_point = ((boxes[i][0]+boxes[i][2])/2, (boxes[i][1]+boxes[i][3])/2)
        ground_position = image_point_to_ground_3d(target_point, camera_matrix, dist_coeffs, camera_height, tilt_angle_deg)

        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = camera_frame

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
    """ Function to host a socket server and send frame + detection data """
    while not rospy.is_shutdown():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((HOST, PORT))
            sock.listen(5)

            print(f"[*] Server started at {HOST}:{PORT}, waiting for a connection...")
            client_socket, client_address = sock.accept()
            print(f"[*] Accepted connection from {client_address}")

            cap = cv2.VideoCapture(4)
            if not cap.isOpened():
                print("[!] Could not open video device")
                break

            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    print("Failed to capture video frame")
                    break

                detections = run_example(frame)

                # --- Encode frame to reduce size before sending ---
                _, encoded_frame = cv2.imencode(".jpg", frame)
                frame_data = encoded_frame.tobytes()

                # --- Pack data into a dictionary ---
                data_packet = {
                    "image": frame_data,
                    "detections": detections
                }

                # --- Serialize and send over socket ---
                serialized_data = pickle.dumps(data_packet)
                message_size = struct.pack("L", len(serialized_data))
                client_socket.sendall(message_size + serialized_data)

                time.sleep(0.03)  # Control sending speed to avoid overloading

        except (BrokenPipeError, ConnectionResetError):
            print("[!] Connection lost. Reconnecting...")
            time.sleep(2)
        except Excetion as e:
            print(f"[ERROR] {e}")
            time.sleep(5)
        finally:
            sock.close()
            cap.release()

def main():
    """ Main function to start OpenCV processing and socket server in parallel """
    current_dir = os.path.dirname(os.path.realpath(__file__))
    model_path = os.path.join(current_dir, "../models/encrypt-model-v1-fp16.7z")
    load_model_data(model_path)

    # Start socket server in a separate thread
    socket_thread = threading.Thread(target=socket_server, daemon=True)
    socket_thread.start()

    rospy.spin()  # Keep ROS running

if __name__ == "__main__":
    main()
p
