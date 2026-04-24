#!/bin/bash

# set camera device nodes
CAMERAS=("/dev/video0" "/dev/video2" "/dev/video4" "/dev/video6")

# set output directory
OUTPUT_DIR="./recordings"
mkdir -p "$OUTPUT_DIR"

# set recording duration in seconds
DURATION=30

# simultaneously record from all cameras
for CAM in "${CAMERAS[@]}"; do
    OUTPUT_FILE="$OUTPUT_DIR/$(basename $CAM).mp4"
    # use buffering to reduce frame loss
    ffmpeg -f v4l2 -input_format mjpeg -video_size 3840x2160 -framerate 60 -i "$CAM" \
        -t "$DURATION" -c:v copy -r 60 "$OUTPUT_FILE" &
done

# wait for all background processes to finish
wait
echo "All recordings completed!"