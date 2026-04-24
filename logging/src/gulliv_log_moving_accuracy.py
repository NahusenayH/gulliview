#!/usr/bin/env python3

import socket
import struct
import numpy
import csv
import datetime

listen_port=2121

detection_message_format_string = '!IIIffI'
message_format_string_header = '!IIIqqII'
heartbeat_format_string = '!IIq4f'

    
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

now = datetime.datetime.now()
log_timestamp = now.strftime("%d_%m_%Y %H_%M_%S")
# Camera names from wall to door:  DOOR  0, 1, 2, 3  BACK WALL

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
                
            except socket.error:
                no_data += 1
                # if flushing has received no value in latest 
                if no_data == return_value:
                     print("Old port data flushed.")
                     return
                pass

sampleNo = 1
amountOfSamples = 2000
SPEED = 0.5
latest_detection_cam = 100
model_str = "simple" # change to "simple" or "advanced"

input(f'GulliView moving accuracy script. Speed is set to {SPEED}. Enter to begin.')

filename = f'{log_timestamp}_{model_str}_spd_{SPEED}_accuracy_1080_moving.csv'
with open(filename, 'w', newline='') as file:
    writer = csv.writer(file, dialect = 'excel')
    writer.writerow(['CameraID','Line','DetectionStatus','FastSearchUsed','Timestamp','x','y'])

# input(f"Press enter to begin logging data for line {line}.")
# print("Running loop...")
sampleNo = 1
while True: 
    for line in [1,2,3]:
        sampleNo = 1
        latest_detection_cam = 100
        input(f"About to log data for line {line} at speed = {SPEED} with {amountOfSamples} samples. Press any key to start.")
        flush_port()
        print("Running loop...")
        while True:
            try:
                while True: # Loop through all messages until the exception is thrown
                    # Receive packet
                    udp_datagram = sock.recv(256)
                    print("new msg,size=",len(udp_datagram))
                    # avoid first 5 detections on startup (camera misses these because of auto-focus)
                    if sampleNo < 5: 
                        sampleNo += 1
                        continue
                    # Unpack packet
                    msg_header, rest = struct.unpack(message_format_string_header, udp_datagram[:header_size]), udp_datagram[header_size:]
                    (msg_type, msg_type2, seq, msecs, fast_search_used, cam_name, length) = msg_header
                    timestamp = msg_header[3]
                    
                    detections = []
                    for _ in range(0,length):
                        detection = struct.unpack(detection_message_format_string, rest[:detection_size])
                        detections.append(detection)
                        rest = rest[detection_size:]

                    # if camera message has no detections and the latest known detection was from that camera
                    if (not detections) and (cam_name == latest_detection_cam):
                        print(f"CAM {cam_name} empty")
                        sampleNo += 1
                        print(f"sample: {sampleNo}")
                        with open(filename, 'a', newline='') as file:
                            writer = csv.writer(file, dialect = 'excel')
                            writer.writerow([cam_name, line, '0', fast_search_used, timestamp, '0', '0']) # log zero for det. status if nothing found

                    else:
                        # loop through all detections and append stuff
                        for i in range(0, length):
                            (tag_id, x, y, theta, speed, camera_id) = detections[i]
                            print(f"tag_id: {tag_id} detected by camera: {cam_name}. Fast search status: {fast_search_used}")
                            # if not currDetections[camera_id]:
                            #     currDetections[camera_id] = True
                            latest_detection_cam = cam_name
                            if fast_search_used:
                                sampleNo += 1
                            print(f"sample: {sampleNo}")
                            with open(filename, 'a',newline='') as file:
                                writer = csv.writer(file, dialect = 'excel')
                                # Add 0s since other stamps use ns time || gammal kommentar, vad menas?
                                writer.writerow([cam_name, line, '1', fast_search_used, f'{timestamp}', x, y])

                    if sampleNo == amountOfSamples:
                        break
                
            except socket.error:
                pass
            if sampleNo == amountOfSamples:
                    break