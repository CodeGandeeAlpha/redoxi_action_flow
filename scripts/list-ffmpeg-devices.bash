#!/bin/bash

# example v4l2-ctl --list-devices
# $ v4l2-ctl --device=/dev/video4 --list-formats-ext
# ioctl: VIDIOC_ENUM_FMT
#         Type: Video Capture

#         [0]: 'MJPG' (Motion-JPEG, compressed)
#                 Size: Discrete 1920x1080
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 3840x2160
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 3840x2880
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 2592x1944
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 2048x1536
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1600x1200
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x960
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x720
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1024x768
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 960x720
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 800x600
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 640x480
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 320x240
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#         [1]: 'YUYV' (YUYV 4:2:2)
#                 Size: Discrete 1920x1080
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 3840x2160
#                         Interval: Discrete 1.000s (1.000 fps)
#                 Size: Discrete 3840x2880
#                         Interval: Discrete 1.000s (1.000 fps)
#                 Size: Discrete 2592x1944
#                         Interval: Discrete 1.000s (1.000 fps)
#                 Size: Discrete 2048x1536
#                         Interval: Discrete 0.333s (3.000 fps)
#                 Size: Discrete 1600x1200
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x960
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1280x720
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 1024x768
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 960x720
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 800x600
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 640x480
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)
#                 Size: Discrete 320x240
#                         Interval: Discrete 0.033s (30.000 fps)
#                         Interval: Discrete 0.040s (25.000 fps)
#                         Interval: Discrete 0.050s (20.000 fps)
#                         Interval: Discrete 0.067s (15.000 fps)
#                         Interval: Discrete 0.100s (10.000 fps)
#                         Interval: Discrete 0.200s (5.000 fps)

# Check if device path is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <device_path>"
    echo "Example: $0 /dev/video0"
    exit 1
fi

DEVICE=$1

# Check if device exists
if [ ! -c "$DEVICE" ]; then
    echo "Error: Device $DEVICE does not exist or is not a character device"
    exit 1
fi

# Get the formats and resolutions using v4l2-ctl
formats=$(v4l2-ctl --device="$DEVICE" --list-formats-ext)

# Check if v4l2-ctl command succeeded
if [ $? -ne 0 ]; then
    echo "Error: Failed to get formats from $DEVICE"
    exit 1
fi

# Process the output
current_format=""
echo "Available formats for $DEVICE:"
echo "==============================="

echo "$formats" | while IFS= read -r line; do
    # Detect format
    if [[ $line =~ \[.*\]:\ \'([A-Z0-9]*)\'.*$ ]]; then
        format_code="${BASH_REMATCH[1]}"
        # Map format codes to ffmpeg format names
        if [[ "${format_code,,}" == "mjpg" || "${format_code,,}" =~ ^mjpe?g ]]; then
            current_format="mjpeg"
        elif [[ "${format_code,,}" == "yuyv" ]]; then
            current_format="yuyv422"
        else
            current_format=$(echo "$format_code" | tr '[:upper:]' '[:lower:]')
        fi
        echo -e "\nFormat: $current_format (original: $format_code)"
        echo "------------------------"
    fi
    
    # Detect resolution
    if [[ $line =~ Size:\ Discrete\ ([0-9]+x[0-9]+) ]]; then
        resolution="${BASH_REMATCH[1]}"
    fi
    
    # Detect fps
    if [[ $line =~ Interval:\ Discrete.*\(([0-9.]+)\ fps\) && -n "$resolution" && -n "$current_format" ]]; then
        fps="${BASH_REMATCH[1]}"
        
        echo "Resolution: $resolution"
        echo "FPS: $fps"
        echo "FFplay show:"
        echo "ffplay -f v4l2 -input_format $current_format -video_size $resolution -framerate $fps -fflags nobuffer -flags low_delay -i $DEVICE"
        echo "FFmpeg pipe:"
        echo "ffmpeg -f v4l2 -input_format $current_format -video_size $resolution -framerate $fps -discard nokey -thread_queue_size 1 -fflags nobuffer -flags low_delay -i $DEVICE -f rawvideo -pix_fmt bgr24 -"
        echo ""
    fi
done

exit 0