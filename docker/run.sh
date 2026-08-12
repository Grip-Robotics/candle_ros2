#!/bin/bash
# Start an interactive shell in the candle_ros2 container with USB access
# to the CANdle adapter. Build the image first:
#   docker build -t candle_ros2 .
set -e
exec docker run -it --rm \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --network host \
    --ipc host \
    candle_ros2 "$@"
