import struct
import socket
import sys
import time
import os
import numpy
import logging
from datetime import date
import posix_ipc
import mmap


REMOVE_THRESHOLD = 10000 # milliseconds
FRESH_THRESHOLD = 3000 # milliseconds

heartbeat_format_string = '!IIq12f'

HEARTBEAT_PERIOD = 1000 # (milliseconds) period for sending heartbeat

MIN_INTERVAL = min(HEARTBEAT_PERIOD, FRESH_THRESHOLD) # (milliseconds) minimum interval for sending recovery message

# shared memory
semaphore = posix_ipc.Semaphore("/my_semaphore", posix_ipc.O_CREAT,initial_value=1)

shared_memory_file = "/dev/shm/my_shared_memory"
fd = os.open(shared_memory_file, os.O_CREAT | os.O_RDWR)
size = 1024
os.ftruncate(fd, size)

shared_memory = mmap.mmap(fd, size)
#shared memory end

def recovery_mode(now, members):
    allFresh = True

    # Remove old members
    for tag_id in list(members.keys()):
        (tag_id, heartbeat_timestamp, last_detection_timestamp) = members[tag_id]

        if ((last_detection_timestamp + REMOVE_THRESHOLD) < now) or ((heartbeat_timestamp + REMOVE_THRESHOLD) < now):
            #print("REMOVING id=", tag_id, "tag_det_time=", last_detection_timestamp, " hrtbt_time=", heartbeat_timestamp)
            del members[tag_id]

    print(list(members.keys()))
    
    # Recovery mode if members is empty
    if len(list(members.keys())) == 0:
        return True, 0

    # Check if remaining members is fresh
    delay_time = 0
    for tag_id in list(members.keys()):
        (tag_id, heartbeat_timestamp, last_detection_timestamp) = members[tag_id]

        if (heartbeat_timestamp + FRESH_THRESHOLD) < now or (last_detection_timestamp + FRESH_THRESHOLD) < heartbeat_timestamp:
            delay_time = now - heartbeat_timestamp
            allFresh = False
            break
    
    return (not allFresh, delay_time)

def getOldestDetections(members):
    oldest_detections = [(-1,-1,-1,-1,-1.0,-1.0,-1),(-1,-1,-1,-1,-1.0,-1.0,-1),(-1,-1,-1,-1,-1.0,-1.0,-1)]
    for sendingRobot in members:
        for taggedRobot in sendingRobot[2]:
            taggedRobotIndex = sendingRobot[2].index(taggedRobot)
            if (taggedRobot[1] < oldest_detections[taggedRobotIndex][1] and taggedRobot[1]!=-1) or oldest_detections[taggedRobotIndex][1]==-1:
                oldest_detections[taggedRobotIndex] = taggedRobot
    
    return oldest_detections

def main():
    # File descriptor for the write end of the pipe
    
    if len(sys.argv) == 5:
        # Get "IP address of Server" and also the "port number" from argument 1 and argument 2
        listen_port = int(sys.argv[1])
        broadcast_addr = sys.argv[2]
        broadcast_port = int(sys.argv[3])
        frequency = float(sys.argv[4])
    else:
        print("Usage : python3 membership_service.py <listen_port> <broadcast_addr> <broadcast_port>")
        exit(1)
        
    print(f"frequency: {frequency}")
    # Make socket and bind to ports as well as making the socket non-blocking
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    sock.setblocking(0)
    sock.bind(('', listen_port))

    # Tuple for each wifibot. Contains: TagID, heartbeat timestamp and latest known detection for each wifibot.
    members = [ (-1,-1,[(-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1)]),
                (-1,-1,[(-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1)]),
                (-1,-1,[(-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1), (-1,-1,-1,-1,-1,-1,-1)])]
    
    # oldest detections. This is default value, until a detection of that wifibot is made
    oldest_detections = [(-1,-1,-1,-1,-1.0,-1.0,-1),(-1,-1,-1,-1,-1.0,-1.0,-1),(-1,-1,-1,-1,-1.0,-1.0,-1)]

    n_true = 0
    n_false = 0

    counter = 0
    start = round(time.time() * 1000)
    #data = "Hello from Python!"
    #with open('my_pipe', 'w') as f:
    #        f.write(data)
    #print("Python wrote to pipe")
    while True:
        while True: # Read messages until empty
            try:
                now = round(time.time()*1000)
                udp_datagram = sock.recv(1024)
                header_size = struct.calcsize('!iqi')
                message, rest = struct.unpack('!iqi', udp_datagram[:header_size]), udp_datagram[header_size:]
                #print("message header unpacked")
                (tag_id, heartbeat_timestamp, length) = message
                
                #Put the detections received from the wifibot into this array, for it to then go into members
                detection_records = []
                for i in range(0,length):
                    detection_record_format_string = '!iqiiffi'
                    #print(f"before unpack detection: {i}")
                    detection = struct.unpack(detection_record_format_string, rest[:struct.calcsize(detection_record_format_string)])
                    #print(detection)
                    detection_records.append(detection)
                    rest = rest[struct.calcsize(detection_record_format_string):]
                #print("recived from Robot: " + str(detection_records[0][1]) + "," + str(detection_records[1][1]) + "," + str(detection_records[2][1] ))
                #print("received heartbeat: now=", now, ", tag_id=", tag_id, ", heartbeat_timestamp=", heartbeat_timestamp, ", detection_records=", detection_records)
                tag_id = tag_id - 4
                members[tag_id] = (tag_id + 4, heartbeat_timestamp, detection_records)
            except:
                break

        now = round(time.time()*1000)
        #recovery, delay_time = recovery_mode(now, members)

        # if recovery:
        #     print(f'delay_thing: {delay_time}')
        #     n_true += 1
        # else:
        #     n_false += 1

        # print(f"now={now}, recovery_mode:{recovery}, ratio_true={n_true}/{n_true+n_false}")
        
        oldest_detections = getOldestDetections(members)
    
        #print(f"sent fom Matser to GV: {oldest_detections[0][1]},{oldest_detections[1][1]}, {oldest_detections[2][1]}")

        packed_recovery = b"".join(struct.pack("!iqiiffi", *t) for t in oldest_detections)


        # write to shared memory
        data = str(oldest_detections).encode()
        try:
            semaphore.acquire(timeout=0.00001)
            shared_memory.seek(0)
            shared_memory.write(b'\0' * size)

            shared_memory.seek(0)
            shared_memory.write(data)
        except posix_ipc.BusyError:
            print("shared memory is busy")
        finally:
            semaphore.release()

        # Send the oldest detection for each wifibot to GulliView
        #print("Send recovery to gv")
        #sock.sendto(packed_recovery, ('224.1.1.1', 2222))
        
        period_ms = 1000/frequency
    
        end_ts = round(time.time()*1000)
        #frequency control function
        counter += 1
        if end_ts-start < counter*period_ms:
            start2 = round(time.time()*1000)
            #time.sleep((counter*period_ms - (end_ts-start))/1000)
            #time.sleep(0.01) # sleep for 0.05 seconds
            #print("Master Slept for: ", round(time.time()*1000) - start2)

if __name__ == "__main__":
    main()
