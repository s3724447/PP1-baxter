# https://hub.docker.com/r/davetcoleman/baxter_simulator/~/dockerfile/
# vicariousinc/baxter-simulator:kinetic
# Run simulated Baxter in Gazebo


FROM osrf/ros:kinetic-desktop-full
MAINTAINER Dave Coleman dave@dav.ee

COPY 02proxy /etc/apt/apt.conf.d/02proxy

# Fix ROS keys
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update apt because previous images clear this cache
# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt update && \
    # Install some base dependencies
    apt install -y \
	iproute2 host wget less sudo vim-tiny iputils-ping

# Fix issue for error when using cameras in gazebo 7(?)
# https://github.com/uzh-rpg/rpg_quadrotor_control/issues/58
# - but this introduces a regression with untucking arms!
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Ensure Ubuntu and ROS up to date
RUN apt-get update && apt -y dist-upgrade

RUN echo force rebuild from here

ENV TERM xterm

# Setup catkin workspace
ENV CATKIN_WS=/root/ws_baxter
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Download source code
RUN wstool init . && \
    wstool merge https://raw.githubusercontent.com/vicariousinc/baxter_simulator/${ROS_DISTRO}-gazebo7/baxter_simulator.rosinstall && \
    wstool update

RUN apt-get -qq update && \
    # Install some base dependencies
    apt-get -qq install -y \
        # Required for installing dependencies
        python-rosdep \
        # Preferred build tool
        python-catkin-tools && \
    # Download all dependencies
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

# Build repo
WORKDIR $CATKIN_WS
ENV PYTHONIOENCODING UTF-8
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out, but not too much
    # such that the Docker log gets too long (another form of timeout)
    catkin build --jobs 16 --limit-status-rate 0.001 --no-notify

#
# For debugging
#

RUN apt-get update && apt -y install ros-kinetic-catkin

#
# Dataspeed mobility base
#

WORKDIR $CATKIN_WS

RUN wget https://bitbucket.org/DataspeedInc/mobility_base_ros/raw/master/mobility_base.rosinstall -O /tmp/mobility_base.rosinstall
RUN wget https://bitbucket.org/DataspeedInc/mobility_base_simulator/raw/master/mobility_base_simulator.rosinstall -O /tmp/mobility_base_simulator.rosinstall

RUN wstool merge -t src /tmp/mobility_base.rosinstall
RUN wstool merge -t src /tmp/mobility_base_simulator.rosinstall

# wstool status needed for proper make?? (https://github.com/vcstools/wstool/issues/77)
RUN wstool status -t src && wstool update -t src

##RUN sudo apt-get -y install ros-indigo-baxter-simulator ros-kinetic-joint-state-controller
RUN sudo apt -y install ros-kinetic-joint-state-controller
RUN rosdep update && rosdep install -y --from-paths src --ignore-src -r

RUN rm -rf build devel
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out, but not too much
    # such that the Docker log gets too long (another form of timeout)
    catkin build --jobs 1 --limit-status-rate 0.001 --no-notify

#
# VXLab extensions...
#

#
# OpenGL + NoVNC (port 6080)
#

USER root
#RUN echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user

#RUN git clone https://github.com/kanaka/noVNC.git /opt/noVNC && \
#  cd /opt/noVNC && \
#  git checkout 6a90803feb124791960e3962e328aa3cfb729aeb && \
#  ln -s vnc_auto.html index.html

# noVNC (http server) is on 6080, and the VNC server is on 5900
EXPOSE 6080 5900

# force repeat COPY
#COPY etc /etc
#COPY usr /usr

ENV DISPLAY :0

WORKDIR /root

RUN apt -qq update && \
    apt install -y ros-kinetic-nav2d


WORKDIR $CATKIN_WS
ADD baxter.sh baxter.sh
#ADD vncpasswd vncpasswd
#RUN groupadd -r vxlab && adduser --disabled-password --ingroup vxlab --gecos '' vxlab
#RUN echo 'vxlab ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers
#ADD vncstart vncstart

# rosie mounted at runtime
RUN echo 'source ~/rosie/rosenv.bash' >> /root/.bashrc

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  ros-kinetic-teleop-twist-keyboard

WORKDIR /root/rosie

#CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
