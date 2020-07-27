# baxter-mobility-base-simdemo (vxlab-blue branch)
## ROS Simulation for Rosie (Baxter + Dataspeed Mobility Base) and Blue (MIR100) in VXLab


## Features:
- simultaneous navigation for both robots
- computer vision for Rosie (Alvar)
- collection of demo scripts mounted under ~/rosie in master container

https://youtu.be/U0TmnjHC2r8

## Prerequisites:
- docker-ce
- docker-compose
- optionally, edit 02proxy to point to a nearby Debian apt-cacher-ng proxy

## Build using:

docker-compose build

## Run:

`docker-compose up` (with output on terminal) or
`docker-compose up -d` (run in background, supressing output)

This will start several containers in the background:
- vxlab-rosie (Simulator "gzserver" and core assets for robots)
- novnc, display2 (X sessions for graphical output in browser)
- vxlab-rosie-nav (Navigation for Rosie)
- vxlab-blue (Navigation etc. for Blue)
- alvar-head (Marker recognition for Rosie's head camera)

To stop:

`docker-compose down -d`

## View simulation:

Point your browser on the simulation host, substituting HOSTNAME: http://HOSTNAME:8080/vnc_auto.html

To view output:

Point your browser at HOSTNAME:8080 (main output on "novnc" display) or HOSTNAME:8081 (secondary output on "display2")

You may need to run the gazebo client manually in the master container if it does not start automatically; see below.

Connect to console of master container:

`docker exec -it vxlab-rosie bash`

Run:

`DISPLAY=novnc:0 gzclient`

Refer to documentation for Gazebo, Baxter, Mobility Base, MIR100, Navigation, etc.

## Debug simulation

Connect to console of main container:

`docker exec -it vxlab-rosie bash`

Run:

`DISPLAY=display2:0 rviz`

Refer to documentation for Rviz. The Displays pane on the left hand side has an "Add" button which is just out of view off the bottom of the screen. You can drag to detach the Displays pane and move it somewhere more convenient.

Press the Add button and explore adding different displays. The key ones are "Map" (by topic) and "Robot model" (by display type). Once these two are added you can also click on "2D Nav Goal", then click on the map to position an arrow for the desired location of the robot. Amusingly, since the map provided is of the actual VXLab, but does not match the simulated environment, Rosie does a plausible job at navigating to a given position, but does not succeed.

## Troubleshooting and notes:

- Refer to documentation for Gazebo and the various ROS components

- Several containers mount a docker volume under "~/rosie". Thus, changes to this directory are persistent and cause changes to the directory with the same name on the container host. Be careful! Take backups!
