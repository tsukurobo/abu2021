FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    curl \
    git \
    vim \
    python-pip \
    lsb-release \
    gnupg

# ROS setting
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    python-rosdep
RUN rosdep init && rosdep update

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python-rosinstall \
    python-catkin-tools

# Python3 setting
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python3 get-pip.py
RUN pip3 install rospkg catkin_pkg

# Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc
