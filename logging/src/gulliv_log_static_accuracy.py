#!/usr/bin/env python3

import socket
import struct
import numpy
import csv
import datetime

"""
Script for logging accuracy from GulliView with static WifiBot in different positions in a grid.
This is to be run on a local machine while on the ROStig network and listens to a specified port.
Also needs a GulliView.cpp modified to publish messages when no tag(s) is/are found.
This was made to work and not much more.
"""

listen_port=2121

detection_message_format_string = '!IIIffI'
message_format_string_header = '!IIIqqII'
# heartbeat_format_string = '!IIq4f'

    
header_size = struct.calcsize(message_format_string_header)
detection_size = struct.calcsize(detection_message_format_string)

def get_socket(listen_port):
    # Make socket
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(0)
    sock.bind(('', listen_port))

    return sock

# Get a socket
sock = get_socket(listen_port)

# Initialize detection matrix
# Rows represent camera, coloumns the robots
detection_matrix = numpy.array([[0,0,0],
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]])

#camera id of the last detection message
detection_camera = 0

# with open('gulliview_detection_log.txt', 'w') as file:
#     file.write('camera,area,timestamp,x,y\n')

sampleNo = 1
amountOfSamples = 2500
now = datetime.datetime.now()
log_timestamp = now.strftime("%d_%m_%Y %H_%M_%S")
# Camera names from door to wall:  DOOR  0, 1, 2, 3  BACK WALL
CAMERA_NAME = 3

filename = f'{log_timestamp}_simple_cam_{CAMERA_NAME}_accuracy_1080_static.csv'

input(f'Camera is set to {CAMERA_NAME}. Correct? Press enter or quit.')

def flush_port():
     return_value = 100
     no_data = 0
     received_packets = 0
     while True: 
            try:
                while True: # Loop through all messages until the exception is thrown
                    # Receive packet
                    udp_datagram = sock.recv(256)
                    received_packets += 1
                    print("new message, size=",len(udp_datagram), "number: ", received_packets)

                    # Unpack packet
                    # msg_header, rest = struct.unpack(message_format_string_header, udp_datagram[:header_size]), udp_datagram[header_size:]
                    # (msg_type, msg_type2, seq, msecs, send_timestamp, _, length) = msg_header
                    # timestamp = msg_header[3]
                    
                    # detections = []
                    # for _ in range(0,length):
                    #     detection = struct.unpack(detection_message_format_string, rest[:detection_size])
                    #     detections.append(detection)
                    #     rest = rest[detection_size:]
                
            except socket.error:
                no_data += 1
                # if flushing has received no value in latest 
                if no_data == return_value:
                     print("Old socket data flushed.")
                     return
                pass

# for i in range (4):
with open(filename, 'w', newline='') as file:
    writer = csv.writer(file, dialect = 'excel')
    writer.writerow(['CameraID','Area','Timestamp','x','y'])

while True:
    for area in range(2,10,3):
        input(f"Press enter to begin logging data for area {area}.\nWHITEBOARD WALL\n1|2|3\n-----\n4|5|6\n-----\n7|8|9")
        sampleNo = 1
        flush_port() # removes old entries from port

        while True: 
            try:
                while True: # Loop through all messages until the exception is thrown
                    # Receive packet
                    udp_datagram = sock.recv(256)
                    print("new message, size=",len(udp_datagram))

                    # avoid first 5 detections on startup (camera misses these because of auto-focus)
                    if sampleNo < 5: 
                        sampleNo += 1
                        continue
                    # Unpack packet
                    msg_header, rest = struct.unpack(message_format_string_header, udp_datagram[:header_size]), udp_datagram[header_size:]
                    (msg_type, msg_type2, seq, msecs, _, cam_name, length) = msg_header
                    timestamp = msg_header[3]
                    
                    print(10*'-')
                    print(f"Lab camera: {cam_name}")
                    print(10*'-')

                    detections = []
                    for _ in range(0,length):
                        detection = struct.unpack(detection_message_format_string, rest[:detection_size])
                        detections.append(detection)
                        rest = rest[detection_size:]


                    if not detections:
                        print("EMPTY\n\n")
                        with open(filename, 'a', newline='') as file:
                            writer = csv.writer(file, dialect = 'excel')
                            writer.writerow(['10',area,timestamp,'0','0'])

                    else:

                        for i in range(0, length):
                            (tag_id, x, y, theta, speed, camera_id) = detections[i]
                            print(f"tag_id: {tag_id} detected by camera: {camera_id}")
                            # currDetections[camera_id] = True

                            with open(filename, 'a',newline='') as file:
                                writer = csv.writer(file, dialect = 'excel')
                                # Add 0s since other stamps use ns time || gammal kommentar, vad menas?
                                writer.writerow([CAMERA_NAME,area,f'{timestamp}',x,y])

                    sampleNo += 1
                    print(sampleNo)
                    if sampleNo == amountOfSamples:
                        break

                if sampleNo == amountOfSamples:
                        break
                
            except socket.error:
                pass