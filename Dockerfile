# ROS 2 Humble (Ubuntu 22.04) image with the candle_ros2 node and MAB's
# candletool CLI for talking to MD drives over a CANdle USB adapter.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        curl \
        libusb-1.0-0-dev \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY . src/candle_ros2

# Build the ROS 2 package (this also builds the bundled CANdle-SDK library)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build candletool separately (the ROS build force-disables it). It is MAB's
# CLI for discovering, configuring, and testing drives without ROS.
RUN cmake -S src/candle_ros2/CANdle-SDK -B /tmp/sdk-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCANDLESDK_BUILD_CANDLETOOL=ON \
        -DCANDLESDK_BUILD_EXAMPLES=OFF && \
    cmake --build /tmp/sdk-build --target candletool -j"$(nproc)" && \
    install -m 755 /tmp/sdk-build/candletool/candletool /usr/local/bin/candletool && \
    mkdir -p /etc/candletool && \
    cp -r src/candle_ros2/CANdle-SDK/candletool/template_package/etc/candletool/. /etc/candletool/ && \
    rm -rf /tmp/sdk-build

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
