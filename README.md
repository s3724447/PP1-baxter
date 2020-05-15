# baxter-mobility-base-simdemo
ROS Sim for Baxter + mobility base in VXLab

https://youtu.be/U0TmnjHC2r8

Requires install of docker-ce on your platform to build and run container.

Experimental navigation stack on "navigation" branch.

Build config:

(optionally, set 02proxy to point to a nearby apt-cacher-ng proxy)

Build using:

docker-compose build

Run using:

docker-compose up -d

This will start three containers in the background: vxlab-rosie (Simulation platform); novnc (X session for graphical output in browser), vxlab-rosie-nav (Navigation).

Simulator:

docker exec -it vxlab-rosie bash

~/rosie/simload &

Graphical output with Gazebo:

Point your browser on the same machine, substituting HOSTNAME: http://HOSTNAME:8080/vnc_auto.html

Once the gazebo world has started up, type:

~/rosie/prepsim

Graphical display:

Point your browser at HOSTNAME:8080

Simulator never appears (with XCB errors on console): try moving the Baxter window, closing your browser, rerunning the simulator

Navigation:

Setup (once only):

cd ~/navigation_ws ; ./deps && ./rosbuild

Run:

~/rosie/navstart

Rviz (navigation/debugging):

To see an rviz window (for debugging), type:

Using rviz requires reading some documentation. The Displays pane on the left hand side has an "Add" button which is just out of view off the bottom of the screen. You can drag to detach the Displays pane and move it somewhere more convenient.

Press the Add button and explore adding different displays. The key ones are "Map" (by topic) and "Robot model" (by display type). Once these two are added you can also click on "2D Nav Goal", then click on the map to position an arrow for the desired location of the robot. Amusingly, since the map provided is of the actual VXLab, but does not match the simulated environment, Rosie does a plausible job at navigating to a given position, but does not succeed.

Design:

The navigation branch mounts configuration scripts as a docker volume under "~/rosie". Thus, changes to this directory are persistent and cause changes to the directory with the same name on the container host. Be careful! Take backups!

Mapping:

Use

./mapping-start

To start the mapping process. (In gazebo, add a map with topic "/map" to see what has been mapped.) In a separate docker bash shell, start a keyboard-based controller for the mobilty base using

./controlstart

Instructions are shown on standard output. To generate a coherent map, use strafe commands only, no rotation. With the current configuration for the mapping component (hector_slam), rotation breaks the SLAM tracking so maps are incoherent.  Finally use

./savemap

to save the map
