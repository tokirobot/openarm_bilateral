# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ros:humble-ros-base

# Install common dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    can-utils \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 setup
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY ./openarm_ros2 /root/ros2_ws/src/ros2_packages
COPY ./openarm_bilateral /root/ros2_ws/src/openarm_bilateral

# Build
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-select ros2_packages openarm_bilateral

# Source by default
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

