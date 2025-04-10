# Use ROS 2 Humble on Ubuntu 22.04
FROM ros:humble

# Install SROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-sros2 \
  && rm -rf /var/lib/apt/lists/*

# Source ROS in the container shell
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Copy your local code into the container
WORKDIR /root/ros-ws
COPY . /root/ros-ws

# Build your ROS 2 packages (colcon)
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install || echo "No colcon build"

CMD ["/bin/bash"]

