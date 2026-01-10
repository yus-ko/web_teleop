FROM ros:humble

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/* && \
    pip install websockets

# ROS2 workspace をビルド
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 環境設定
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run websocket_receiver websocket_receiver"]
