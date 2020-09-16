# baxter-mobility-base-simdemo (vxlab-blue branch)
## ROS Simulation for Rosie (Baxter + Dataspeed Mobility Base) and Blue (MIR100) in VXLab

## Features:
- simultaneous navigation for both robots
- computer vision for Rosie (Alvar)
- collection of demo scripts mounted under ~/rosie in master container

[![Rosie and Blue Navigating in VXLab](https://img.youtube.com/vi/Jtl_j8n0Mf8/0.jpg)](https://www.youtube.com/watch?v=Jtl_j8n0Mf8)

(Recommended branch: blue-mir100)

## Prerequisites:
- docker-ce
- docker-compose
- (optional mir100 in same sim: https://github.com/ipeakermit/mir_robot_in_vxlab_sim)
- optionally, edit 02proxy to point to a nearby Debian apt-cacher-ng proxy

## Build using:

docker-compose build

## Run:

./create-vxlab-network

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

## Master container demos / commands:

To connect to the console of master container:

`docker exec -it vxlab-rosie bash`

You can do this multiple times from multiple bash/terminal sessions.

(If the gazebo simulation client output (`gzclient`) has not already appeared, run:

`gzclient &`

Demos for Rosie (`untuck` command may be required first to enable motors running):
- Arm setup (once per session): `./untuck` (and optionally `./tuck`)
- Mobility base movement (custom):
  - `./leftrotate.py` and `./rightrotate.py`
  - `./left.py` and `./right.py`
  - `./forward.py`and `./back.py`
- Pre-existing Baxter function beginning with: `rosrun baxter_examples <TAB>` or `rosrun baxter_tools <TAB>` (refer to Baxter documentation)
- Arm movement under keyboard control: `rosrun baxter_examples joint_position_keyboard.py`
- Arm movement using Trac inverse kinematics solver: `cd ikeg ; ./tractest.py` 

Refer to documentation for Gazebo, Baxter, Mobility Base, MIR100, Navigation, etc.

## Debug output (Rviz)

Connect to console of main container (see elsewhere) and run:

`rviz &`

Refer to documentation for Rviz. The Displays pane on the left hand side has an "Add" button which is just out of view off the bottom of the screen. You can drag to detach the Displays pane and move it somewhere more convenient.

Press the Add button and explore adding different displays. The key ones are "Map" (by topic) and "Robot model" (by display type). Once these two are added you can also click on "2D Nav Goal", then click on the map to position an arrow for the desired location of the robot. Refer to ROS documentation for navigation.

## Marker recognition (Alvar, Rviz)

Move Rosie to where the head camera is pointed at the large block with the markers. The marker should appear in Rviz as a blue square in roughly the appropriate position.

## MIR100 ("Blue")

Whilst in the Rosie master container, run:

`./blue-minimal` (for model)

then open a new terminal and run:

`docker exec -it vxlab-blue bash`

then
```
cd ~/mir100/blue
./blue_start
```

A separate rviz is required for blue, launched from the vxlab-blue container. To do this, run:

`cd ~/mir100/blue`

`./blue_rviz`

## Notes:

- On startup, especially first startup, you may need to interrupt and restart a graphical application such as gzclient a few times before it first draws correctly. This may include a stream of error messages such as:
`QXcbConnection: XCB error: 2 (BadValue), sequence: 1555, resource id: 1200, major code: 130 (Unknown), minor code: 3`
Some improvements are suggested at: http://wiki.ros.org/docker/Tutorials/GUI

- Several containers mount a docker volume under "~/rosie". Thus, changes to this directory are persistent and cause changes to the directory with the same name on the container host. Be careful! Take backups!

## Human Detector

Currently only functional with fake human detection from a .csv First, start blue with navigation etc. with the above commands, also run RVIZ. Then:

`docker exec -it vxlab-blue bash`

`source /ws/devel/setup.bash`

`roslaunch target_object_detector target_object_detector.launch`

Then in another terminal window:

`docker exec -it vxlab-blue bash`

`source /ws/devel/setup.bash`

`~/mir100/blue/overwrite_targetlist`

`rosrun fake_target_detector fake_target_detector_node`

You will be able to see the fake human detection in RVIZ.

Detected humans (even fake ones) are published to Rostopic "/recognized_result".


