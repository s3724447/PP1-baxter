#!/bin/bash
ROS_MASTER_IP=`~/rosie/hostlookup vxlab-blue`
ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311

while true; do
    # Little Room
    ~/rosie/scripts/go_to.py blue -5.5 -4.0 0.0 1.0
    sleep 5

    # Window
    ~/rosie/scripts/go_to.py blue -5.0 3.0 0.7 0.7
    sleep 5

    # Govlab
    ~/rosie/scripts/go_to.py blue 4.5 -2.0 -0.7 0.7
    sleep 5

    # Door
    ~/rosie/scripts/go_to.py blue 0.5 5.0 1.0 0.0
    sleep 5
done
