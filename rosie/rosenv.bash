#!/bin/bash

HOSTNAME=`hostname`
export ROS_IP=`host ${HOSTNAME} | sed 's/.* //g'`
export MASTER_IP=`host vxlab-rosie | sed 's/.* //g'`
export ROS_MASTER_URI=http://$MASTER_IP:11311
export GAZEBO_MODEL_PATH=~/rosie

export MB_LASER_BIRDCAGE_R2000=1
export MB_LASER_BIRDCAGE_R2000_FREQ=50
export MB_LASER_BIRDCAGE_R2000_SAMPLES=3600

export LIBGL_ALWAYS_INDIRECT=0

#source ~/ws_baxter/devel/setup.bash
#source ~/mb_ws/devel/setup.bash
source /root/mir_ws/devel/setup.bash

#source ~/navigation_ws/devel/setup.bash

