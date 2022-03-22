import socket
import pickle
import struct
import cv2

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn.connect(("10.20.36.6", 6666))

payload_size = struct.calcsize("H")

data = b''
payload_size = struct.calcsize("L")

while True:
    # Retrieve message size
    while len(data) < payload_size:
        data += conn.recv(4096)

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("L", packed_msg_size)[0]

    # Retrieve all data based on message size
    while len(data) < msg_size:
        data += conn.recv(4096)

    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Extract frame
    frame = pickle.loads(frame_data)

    # Display
    cv2.imshow('frame', frame)
    cv2.waitKey(1)