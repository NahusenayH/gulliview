# #!/bin/bash

# # 设置摄像头设备节点
# CAMERAS=("/dev/video0" "/dev/video2" "/dev/video4" "/dev/video6")

# # 设置输出文件路径
# OUTPUT_DIR="./recordings"
# mkdir -p "$OUTPUT_DIR"

# # 录制时长（秒）
# DURATION=60

# # 同时启动多个录制任务
# for CAM in "${CAMERAS[@]}"; do
#     OUTPUT_FILE="$OUTPUT_DIR/$(basename $CAM).mp4"
#     ffmpeg -f v4l2 -i "$CAM" -t "$DURATION" -s 3840x2160 -r 30 "$OUTPUT_FILE" &
# done

# # 等待所有录制完成
# wait
# echo "All recordings completed!"


#!/bin/bash

# 设置摄像头设备节点
CAMERAS=("/dev/video0" "/dev/video2" "/dev/video4" "/dev/video6")

# 设置输出文件路径
OUTPUT_DIR="./recordings"
mkdir -p "$OUTPUT_DIR"

# 录制时长（秒）
DURATION=20

# 同时启动多个录制任务
for CAM in "${CAMERAS[@]}"; do
    OUTPUT_FILE="$OUTPUT_DIR/$(basename $CAM).mp4"
    # 使用缓冲以减少帧丢失
    ffmpeg -f v4l2 -input_format mjpeg -video_size 3840x2160 -framerate 30 -i "$CAM" \
        -t "$DURATION" -c:v copy -r 30 "$OUTPUT_FILE" &
done

# 等待所有录制完成
wait
echo "All recordings completed!"
