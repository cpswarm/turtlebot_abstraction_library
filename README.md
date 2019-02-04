# Turtlebot Abstraction Library
CPSwarm abtraction library for Kobuki Turtlebot 2

## Compile from source
### Source your environment
```
source /opt/ros/<ros-distro>/setup.bash
```
### Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
```
### Get the codes
```
git clone https://github.com/cpswarm/turtlebot_abstraction_library/
```
### Build
```
cd ~/catkin_ws/
catkin_make
```
### Configure
There is a yaml file (param/compute_cost_param.yaml) to configure the parameters of the package:

map_pose_topic:     amcl_pose                        # Topic for receiving the position of the CPS in map coordinates.

target_cost_topic:  cps_selection                    # Topic to send the computed cost to reach target.

cps_selected_topic: bridge/events/cps_selected       # Topic to receive selection response

UUID_topic:         bridge/uuid                      # Topic to receive node UUID 

move_base_action:   move_base                        # Topic to send the robot to somewhere


### Run
```
roslaunch turtlebot_compute_cost compute_cost.launch
```
