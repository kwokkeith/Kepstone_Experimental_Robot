import socket
import pickle
import struct
import cv2
import numpy as np

HOST = "192.168.0.151"
PORT = 5555

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

while True:
    # Receive the message size
    data_size = client_socket.recv(4)
    if not data_size:
        break
    message_size = struct.unpack("!I", data_size)[0]

    # Receive the actual data
    data = b""
    while len(data) < message_size:
        packet = client_socket.recv(4096)
        if not packet:
            break
        data += packet

    # Deserialize the received data
    data_packet = pickle.loads(data)

    # Decode and display the image
    image_data = np.frombuffer(data_packet["image"], dtype=np.uint8)
    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

    cv2.imshow("Received Frame", frame)

    # Print detection results
    for detection in data_packet["detections"]:
        print(f"Detected {detection['class']} at :")
        print(f"Ground position: {detection['ground_position']}")
        print(f"Confidence: {detection['score']}")
        print("-----------------")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

client_socket.close()
cv2.destroyAllWindows()

