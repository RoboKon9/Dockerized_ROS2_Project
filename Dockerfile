#Ubuntu 22.04 base image
FROM ubuntu:22.04

#Interactive prompts during package installs

ENV DEBIAN_FRONTEND=noninteractive

#Install base packages and setup locales
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean

#Set environment variables for locale
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

#Add ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list


RUN apt-get update && apt-get install -y \
    ros-humble-desktop \ #install ROS 2 Humble (desktop variant)
    python3-argcomplete \ #shell auto-completion
    python3-colcon-common-extensions\ #colcon 
    libeigen3-dev\ #install C++ Eigen library for linear algebra
    libyaml-cpp-dev\ #install yaml-c++ library
    ros-humble-rviz-visual-tools\ #install rviz_visual_tools
    && apt-get clean



# Source ROS setup when a new shell starts
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]

