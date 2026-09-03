# Script for starting all 4 cameras together with the membership service.
# Put this in the build folder

DETECTIONS_PORT=2020
HEARTBEAT_PORT=2121
RECOVERY_PORT=2222

TRANSMISSION_RATE=9

if ! [ $# -eq 0 ] 2> /dev/null && ! [ $# -eq 3 ];
then
    cat << EOF
Usage: $0 [detections_port heartbeat_port recovery_port]
    
    Either provide no arguments or all three.

    detections_port:    The port GulliView will broadcast its detections to. Default is 2020.
    heartbeat_port:     The port membership_service.py will listen to for heartbeats from robots. Default is 2121.
    recovery_port:      The port that will be used to send recovery messages from membership_service to GulliView. Default is 2222.
    shared_memory_x: The name of the shared memory that GulliView will use to share detections with the Sender.
    shared_semaphore_x: The name of the shared semaphore that GulliView will use to share detections with the Sender.
EOF
    exit -1
fi

if [ $# -eq 3 ]
then
    DETECTIONS_PORT=$1
    HEARTBEAT_PORT=$2
    RECOVERY_PORT=$3
fi

# Start timer
start_time=$(date +%s)

trap 'kill %1;kill %2;kill %3;kill %4;kill %5;kill $(pgrep -f membership_service.py); kill $(pgrep -f "Transmitter")' INT

python3 membership_service.py $HEARTBEAT_PORT 224.1.1.1 $RECOVERY_PORT $TRANSMISSION_RATE &
PID_MEMBERSHIP=$!

sleep 1
# ./GulliView -d 4 -f tag36h11 -W 1920 -H 1080 -V 192.168.50.255 -B -N my_semaphore -T shared_memory &   # 4 cameras
./GulliView -d 4 -f tag36h11 -W 3840 -H 2160 -V 192.168.50.255 -B -N my_semaphore -T shared_memory &   # 4 cameras
PID_GULLIVIEW=$!

sleep 2
./Transmitter & 
PID_TRANSMITTER=$!

# Wait for GulliView to exit (whether normally or due to a crash)
wait $PID_GULLIVIEW

# If GulliView has exited, kill the other processes
kill %1  # Kill membership_service
kill %2  # Kill any other background processes, adjust accordingly
kill %3  # Adjust as needed
kill %4  # Adjust as needed
kill %5  # Adjust as needed
kill $(pgrep -f membership_service.py)
kill $(pgrep -f "Transmitter")

# End timer
end_time=$(date +%s)

# Calculate elapsed time
elapsed_time=$((end_time - start_time))

echo "Elapsed time: ${elapsed_time} seconds"