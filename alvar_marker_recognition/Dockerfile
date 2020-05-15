FROM vxlab-red

USER root

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  telnet curl wget

ENV CATKIN_WS=/root/ws_alvar
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src

RUN git clone https://github.com/ros-perception/ar_track_alvar.git

WORKDIR $CATKIN_WS
COPY camera_controlx.py src/ar_track_alvar/ar_track_alvar/src/camera_controlx.py
ENV PYTHONIOENCODING UTF-8
RUN source /root/ws_baxter/devel/setup.bash ; catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Status rate is limited so that just enough info is shown to keep Docker from timing out, but not too much
    # such that the Docker log gets too long (another form of timeout)
    catkin build --jobs 1 --limit-status-rate 0.1 --no-notify
COPY alvar_baxter_vxlab.launch alvar_baxter_vxlab.launch
COPY start start

WORKDIR /root
COPY rosenv rosenv

RUN echo "source ~/rosenv" >> ~/.bashrc

COPY 02proxy /etc/apt/apt.conf.d/02proxy

# PROMOTE

# Fix ROS keys
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  iproute2 host

# Launch file is here and referred to as "./"
WORKDIR $CATKIN_WS

#CMD ["/bin/bash","-c","/root/ws_alvar/start"]
#ENTRYPOINT [ "/bin/bash", "-c", "/root/ws_alvar/start" ]
ENTRYPOINT [ "/root/ws_alvar/start" ]
